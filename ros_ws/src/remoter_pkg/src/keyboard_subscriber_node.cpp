#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

class KeyboardVelocityController {
public:
    KeyboardVelocityController() : linear_vel_(0.0), angular_vel_(0.0) {
        // 从参数服务器读取速度上限（支持 launch 文件配置）
        ros::NodeHandle private_nh("~"); // 私有句柄，用于读取私有参数
        private_nh.param("max_linear_velocity", max_linear_vel_, 0.3);
        private_nh.param("max_angular_velocity", max_angular_vel_, 0.2);

        // 订阅键盘输入
        sub_ = nh_.subscribe("/keyboard_state", 1, &KeyboardVelocityController::keyboardCallback, this);

        // 发布速度命令（标准 ROS 速度消息）
        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        ROS_INFO("Keyboard velocity controller started.");
        ROS_INFO("Max Linear Vel: %.2f m/s, Max Angular Vel: %.2f rad/s", max_linear_vel_, max_angular_vel_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_cmd_vel_;


    double linear_vel_;
    double angular_vel_;
    double max_linear_vel_;
    double max_angular_vel_;

void keyboardCallback(const std_msgs::String::ConstPtr& msg) {
    std::string key = msg->data;

    // W: 增加线速度
    if (key == "W") {
        linear_vel_ += 0.03;
        if (linear_vel_ > max_linear_vel_) linear_vel_ = max_linear_vel_;
        ROS_INFO(">> FORWARD: linear_vel = %.3f", linear_vel_);
    }
    // S: 减少线速度（不能倒车，最低为0）
    else if (key == "S") {
        linear_vel_ -= 0.03;
        if (linear_vel_ < 0.0) linear_vel_ = 0.0;
        ROS_INFO(">> SLOW DOWN: linear_vel = %.3f", linear_vel_);
    }
    // A: 增加负角速度（左转）
    else if (key == "A") {
        angular_vel_ -= 0.02;
        if (angular_vel_ < -max_angular_vel_) angular_vel_ = -max_angular_vel_;
        ROS_INFO(">> TURN LEFT: angular_vel = %.3f", angular_vel_);
    }
    // D: 增加正角速度（右转）
    else if (key == "D") {
        angular_vel_ += 0.02;
        if (angular_vel_ > max_angular_vel_) angular_vel_ = max_angular_vel_;
        ROS_INFO(">> TURN RIGHT: angular_vel = %.3f", angular_vel_);
    }
    // P: 急停，速度归零
    else if (key == "P") {
        linear_vel_ = 0.0;
        angular_vel_ = 0.0;
        ROS_INFO(">> EMERGENCY STOP: velocity reset to zero.");
    }
    // Q: 仅角速度归零（用于摆正方向）
    else if (key == "Q") {
        angular_vel_ = 0.0;
        ROS_INFO(">> STRAIGHTEN: angular_vel reset to zero. Linear vel = %.3f", linear_vel_);
    }
    // E: 仅线速度归零（用于停止前进，保持转向）
    else if (key == "E") {
        linear_vel_ = 0.0;
        ROS_INFO(">> HALT FORWARD: linear_vel reset to zero. Angular vel = %.3f", angular_vel_);
    }
    else {
        ROS_INFO(">> Unknown key: %s", key.c_str());
        return; // 不发布新速度
    }

    // 发布新的速度命令
    publishVelocity();
}

    void publishVelocity() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_vel_;
        cmd_vel.angular.z = angular_vel_;
        pub_cmd_vel_.publish(cmd_vel);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_subscriber_node");
    KeyboardVelocityController controller;
    ros::spin();
    return 0;
}