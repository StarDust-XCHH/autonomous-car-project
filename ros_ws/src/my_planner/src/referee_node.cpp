#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>

// ===== 全局常量配置（可在此调整）=====
const double GOAL_X = 1.94;      // 终点 x 坐标  x=1.8,y=2.07
const double GOAL_Y = 2.06;      // 终点 y 坐标
const double GOAL_YAW = 0.0;    // 终点朝向（弧度）
const double WAIT_DURATION = 3.0; // 到达终点后等待时间（秒）
// ===================================

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sendGoalAndWait(MoveBaseClient& ac, double x, double y, double yaw)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";   
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();

    ROS_INFO("Sending goal: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
    ac.sendGoal(goal);
    ac.waitForResult();

    if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("Failed to reach goal!");
        exit(1);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "referee_node");

    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // === Step 1: 记录出发时间，前往终点 ===
    ros::Time start_time = ros::Time::now();
    sendGoalAndWait(ac, GOAL_X, GOAL_Y, GOAL_YAW);
    ros::Time arrival_time = ros::Time::now();

    double forward_duration = (arrival_time - start_time).toSec();
    ROS_INFO("Arrived at goal. Forward time: %.3f seconds", forward_duration);

    // === Step 2: 等待指定时间 ===
    ROS_INFO("Waiting for %.1f seconds...", WAIT_DURATION);
    ros::Duration(WAIT_DURATION).sleep();

    // === Step 3: 返回原点 (0,0,0) ===
    ros::Time return_start = ros::Time::now();
    sendGoalAndWait(ac, 0.0, 0.0, 0.0);
    ros::Time return_end = ros::Time::now();

    double return_duration = (return_end - return_start).toSec();
    double total_duration = forward_duration + WAIT_DURATION + return_duration;

    // === Step 4: 输出最终结果 ===
    ROS_WARN("Mission Summary:");
    ROS_WARN("  Forward time: %.3f s", forward_duration);
    ROS_WARN("  Wait time:    %.3f s", WAIT_DURATION);
    ROS_WARN("  Return time:  %.3f s", return_duration);
    ROS_WARN("  Total time:   %.3f s", total_duration);

    return 0;
}