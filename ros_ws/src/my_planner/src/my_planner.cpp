#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav_core::BaseLocalPlanner)


// PID控制所需变量
double angular_error_pid = 0; // 当前误差
double last_error_pid = 0; // 上一次误差
double error_sum_pid = 0; // 误差累积（积分项）
double error_diff_pid = 0; // 误差变化率（微分项）
double output_pid = 0; // PID输出值



namespace my_planner
{

MyPlanner::MyPlanner()
{
setlocale(LC_ALL, "");
}

MyPlanner::~MyPlanner()
{
}

tf::TransformListener* tf_listener_;

void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
ROS_WARN("局部规划器启动! ");
tf_listener_ = new tf::TransformListener();
}

std::vector<geometry_msgs::PoseStamped> global_plan_;
int target_index_;
bool pose_adjusting_;
bool goal_reached_;

bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
target_index_ = 0;
global_plan_ = plan;
pose_adjusting_ = false;
goal_reached_ = false;
return true;
}

bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
int final_index = global_plan_.size() - 1;
geometry_msgs::PoseStamped pose_final;
global_plan_[final_index].header.stamp = ros::Time(0);
geometry_msgs::PoseStamped pose_base;
tf_listener_->transformPose("base_link", global_plan_[final_index], pose_final);


if (pose_adjusting_ == false)
{
double dx = pose_final.pose.position.x;
double dy = pose_final.pose.position.y;
double dist = std::sqrt(dx * dx + dy * dy);
if (dist < 0.1) // 距离最终终点大于0.1m，继续进行路线追踪，否则进行姿态调整
pose_adjusting_ = true;
}

if (pose_adjusting_ == true)
{
// double final_yaw = tf::getYaw(pose_final.pose.orientation);
// ROS_WARN("调整最终姿态, final_yaw = %.2f", final_yaw);

// // 控制参数
// double desired_linear = pose_final.pose.position.x * 1.5;
// double desired_angular = final_yaw * 0.5;

// // 速度限制（可根据机器人性能调整）
// const double MAX_LINEAR_VEL = 0.05; // m/s
// const double MAX_ANGULAR_VEL = 0.5; // rad/s

// cmd_vel.linear.x = std::max(-MAX_LINEAR_VEL, std::min(desired_linear, MAX_LINEAR_VEL));
// cmd_vel.angular.z = std::max(-MAX_ANGULAR_VEL, std::min(desired_angular, MAX_ANGULAR_VEL));

// if(abs(final_yaw)<0.1){
// goal_reached_ = true;
// ROS_WARN("已到达目标点");
// cmd_vel.linear.x = 0.0;
// cmd_vel.angular.z = 0.0;
// }
// return true;




// -----------------------------------------------------------------
// 以上为原有正速度微调逻辑
// 以下为修改允许负速度微调逻辑,且添加最终死区
// -----------------------------------------------------------------


double dx = pose_final.pose.position.x;
double dy = pose_final.pose.position.y;
double final_yaw = tf::getYaw(pose_final.pose.orientation);

ROS_WARN("调整最终姿态: dx=%.3f, dy=%.3f, yaw=%.3f", dx, dy, final_yaw);

// ====== 新增：死区（Deadband）处理 ======
const double POS_DEADBAND = 0.015; // 1.5 cm
const double YAW_DEADBAND = 0.04; // ~2.3 degrees

double desired_linear = 0.0;
double desired_angular = 0.0;

// 仅当误差超出死区时才输出控制量
if (std::fabs(dx) > POS_DEADBAND) {
desired_linear = dx * 0.8; // 降低增益！从 1.5 → 0.8
}
if (std::fabs(final_yaw) > YAW_DEADBAND) {
desired_angular = final_yaw * 0.6; // 降低增益！从 0.8 → 0.6
}

// 速度限制（更保守）
const double MAX_LINEAR_FORWARD = 0.04;
const double MAX_LINEAR_BACKWARD = -0.04;
const double MAX_ANGULAR = 0.25; // 降低角速度上限

if (desired_linear > 0) {
cmd_vel.linear.x = std::min(desired_linear, MAX_LINEAR_FORWARD);
} else {
cmd_vel.linear.x = std::max(desired_linear, MAX_LINEAR_BACKWARD);
}
cmd_vel.angular.z = std::max(-MAX_ANGULAR, std::min(desired_angular, MAX_ANGULAR));

// ====== 收敛判断：使用更宽松的阈值 + 持续静止判断（可选）======
// ====== 收敛判断：使用更宽松的阈值 + 持续静止判断（可选）======
if (std::fabs(dx) < 0.5 &&
std::fabs(dy) < 0.5 &&
std::fabs(final_yaw) < 6.28) {

goal_reached_ = true;
ROS_WARN("✅ 精确到达目标点，停止调整！");
cmd_vel.linear.x = 0.0;
cmd_vel.angular.z = 0.0;
}

return true;
}

geometry_msgs::PoseStamped target_pose;
for (int i = target_index_; i < global_plan_.size(); i++)
{
geometry_msgs::PoseStamped pose_base;
global_plan_[i].header.stamp = ros::Time(0);
tf_listener_->transformPose("base_link", global_plan_[i], pose_base);

double dx = pose_base.pose.position.x;
double dy = pose_base.pose.position.y;
double dist = std::sqrt(dx * dx + dy * dy);

if (dist > 0.25)
{
target_pose = pose_base;
target_index_ = i;
ROS_WARN("选择第 %d 个路径点作为临时目标，距离=%.2f", target_index_, dist);
break;
}

// 最后一个路径点处理：如果没有找到合适的目标点，则取最后一个点
if (i == global_plan_.size() - 1)
target_pose = pose_base;
}








// ---------------以下是线性速度and角速度逻辑------------------

// // 原始控制律
// double desired_linear = target_pose.pose.position.x * 1.5;
// double desired_angular = target_pose.pose.position.y * 5.0;

// // 设置最大速度限制（可根据机器人性能调整）
// const double MAX_LINEAR_VEL = 0.05; // m/s
// const double MAX_ANGULAR_VEL = 1.0; // rad/s

// // 限制幅度
// cmd_vel.linear.x = std::max(-MAX_LINEAR_VEL, std::min(desired_linear, MAX_LINEAR_VEL));
// cmd_vel.angular.z = std::max(-MAX_ANGULAR_VEL, std::min(desired_angular, MAX_ANGULAR_VEL);



// ---------------以上是线性速度and角速度逻辑------------------



// -----------------------------------------------------------------
// -----------------------------------------------------------------



// // ---------------以下是动态角速度and线速度规划逻辑------------------

// ------------------ 【增强版】双向速度控制：防鬼倒车 + 防鬼前进 ------------------

double dx = target_pose.pose.position.x;
double dy = target_pose.pose.position.y;

// ====== Step 1: 长预瞄路径分析（双向判断） ======
const double LONG_LOOKAHEAD = 0.5;
bool should_reverse = false; // 路径是否主要在后方 → 应倒车
bool should_forward = false; // 路径是否主要在前方 → 应前进

int points_behind = 0, points_ahead = 0, total_points = 0;

for (int i = target_index_; i < global_plan_.size(); i++) {
geometry_msgs::PoseStamped pose_base;
global_plan_[i].header.stamp = ros::Time(0);
try {
tf_listener_->transformPose("base_link", global_plan_[i], pose_base);
double lx = pose_base.pose.position.x;
double ly = pose_base.pose.position.y;
double ldist = std::hypot(lx, ly);

if (ldist > 0.05 && ldist < LONG_LOOKAHEAD) {
total_points++;
if (lx < -0.05) points_behind++;
if (lx > 0.05) points_ahead++;
}
if (ldist >= LONG_LOOKAHEAD) break;
} catch (...) {
continue;
}
}

// 判定逻辑：互斥但可共存于“不确定区”
if (total_points > 2) {
should_reverse = ((double)points_behind / total_points > 0.7);
should_forward = ((double)points_ahead / total_points > 0.7);
}

// 注意：正常路径不会同时满足 should_reverse 和 should_forward，
// 但如果路径弯曲或目标很近，可能都不满足（此时保持当前状态）

// ====== Step 2: 双向状态机 + 帧确认防抖 ======
static bool current_mode_is_reverse = false; // false = 前进, true = 倒车
static int confirm_count = 0;
const int CONFIRM_FRAMES = 3;

bool target_mode_is_reverse = current_mode_is_reverse; // 默认保持

// 决策：什么情况下允许切换？
if (current_mode_is_reverse) {
// 当前在倒车：只有路径明确显示“应前进”时，才考虑切回前进
if (should_forward) {
target_mode_is_reverse = false; // 想切到前进
} else {
target_mode_is_reverse = true; // 继续倒车（即使路径不明确，也不乱切）
}
} else {
// 当前在前进：只有路径明确显示“应倒车”时，才考虑切到倒车
if (should_reverse) {
target_mode_is_reverse = true; // 想切到倒车
} else {
target_mode_is_reverse = false; // 继续前进
}
}

// 状态切换需连续确认
if (target_mode_is_reverse != current_mode_is_reverse) {
confirm_count++;
if (confirm_count >= CONFIRM_FRAMES) {
current_mode_is_reverse = target_mode_is_reverse;
confirm_count = 0;
}
} else {
confirm_count = 0; // 重置计数（避免累积误触发）
}

// ====== Step 3: 构造等效前向目标点（用于控制律） ======
double eff_dx, eff_dy;
if (current_mode_is_reverse) {
eff_dx = -dx;
eff_dy = -dy;
} else {
eff_dx = dx;
eff_dy = dy;
}

// ====== Step 4: 修正后的偏航角限幅逻辑（ANGLE_ZERO_SPEED = 90°） ======
double effective_angle_error = std::fabs(std::atan2(eff_dy, eff_dx));

const double ANGLE_FULL_SPEED = 0.0 * M_PI / 180.0; // 0° in radians
const double ANGLE_ZERO_SPEED = 90.0 * M_PI / 180.0; // 90° in radians
const double N = 1.5; // 控制曲线陡峭程度，建议 3.0 ~ 5.0

double scale = 1.0;

if (effective_angle_error >= ANGLE_ZERO_SPEED) {
scale = 0.0;
} else if (effective_angle_error <= ANGLE_FULL_SPEED) {
scale = 1.0;
} else {
// 归一化到 [0, 1]
double t = (effective_angle_error - ANGLE_FULL_SPEED) / (ANGLE_ZERO_SPEED - ANGLE_FULL_SPEED);
// 使用 1 - t^N 实现：0°附近平缓，90°附近陡降
scale = 1.0 - std::pow(t, N);
// 安全钳位（理论上不需要，但防御性编程）
if (scale < 0.0) scale = 0.0;
if (scale > 1.0) scale = 1.0;
}


// ====== Step 5: 控制律 ======
double desired_linear_eff = eff_dx * 3.5 * scale; // 增益不要高于4，否则危险



// double desired_angular = eff_dy * 7.5;

double kp = 5.5;
double ki = 0.0;
double kd = 0.0;

angular_error_pid = eff_dy;
error_sum_pid += angular_error_pid;
error_diff_pid = angular_error_pid - last_error_pid;
output_pid = kp * angular_error_pid + ki * error_sum_pid + kd * error_diff_pid;
double desired_angular = output_pid;
last_error_pid = angular_error_pid;



double desired_linear = current_mode_is_reverse ? -desired_linear_eff : desired_linear_eff;


// ====== 速度限制 ======
const double MAX_LINEAR_VEL = 0.8;
const double MIN_LINEAR_VEL = -0.8;
const double MAX_ANGULAR_VEL = 1; // 再大就比较危险了，很容易花图

cmd_vel.linear.x = desired_linear;
if (cmd_vel.linear.x > MAX_LINEAR_VEL) cmd_vel.linear.x = MAX_LINEAR_VEL;
if (cmd_vel.linear.x < MIN_LINEAR_VEL) cmd_vel.linear.x = MIN_LINEAR_VEL;
ROS_WARN("Final linear velocity (cmd_vel.linear.x: %f", cmd_vel.linear.x);

cmd_vel.angular.z = desired_angular;
if (cmd_vel.angular.z > MAX_ANGULAR_VEL) cmd_vel.angular.z = MAX_ANGULAR_VEL;
if (cmd_vel.angular.z < -MAX_ANGULAR_VEL) cmd_vel.angular.z = -MAX_ANGULAR_VEL;
ROS_WARN("Final angular velocity (cmd_vel.angular.z): %f", cmd_vel.angular.z);


// ---------------------------------------------------------

// ---------------以上是动态角速度and线速度规划逻辑------------------






cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0)); // 创建黑色图像

for (int i = 0; i < global_plan_.size(); i++)
{
geometry_msgs::PoseStamped pose_base;
global_plan_[i].header.stamp = ros::Time(0);
tf_listener_->transformPose("base_link", global_plan_[i], pose_base);
int cv_x = 300 - pose_base.pose.position.y * 100;
int cv_y = 300 - pose_base.pose.position.x * 100;
cv::circle(plan_image, cv::Point(cv_x, cv_y), 1, cv::Scalar(255, 0, 255)); // 画紫色点
}
cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0), 1);
cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0), 1);

cv::namedWindow("Plan"); // 创建窗口
cv::imshow("Plan", plan_image); // 显示图像
cv::waitKey(1); // 等待1毫秒（关键行）
return true;
}

bool MyPlanner::isGoalReached()
{
return goal_reached_;
}

} // namespace my_planner

