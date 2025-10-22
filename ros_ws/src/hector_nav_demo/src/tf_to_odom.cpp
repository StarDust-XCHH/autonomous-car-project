#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h> // for getYaw

class TfToOdom {
public:
    TfToOdom() : nh_("~") {
        // 从参数服务器获取参数（可选，这里写死）
        base_frame_ = "base_link";
        odom_frame_ = "odom";
        publish_rate_ = 20.0; // Hz

        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                &TfToOdom::timerCallback, this);
    }

    void timerCallback(const ros::TimerEvent&) {
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform("map", base_frame_, ros::Time(0), ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "Failed to lookup transform from map to %s: %s", base_frame_.c_str(), ex.what());
            return;
        }

        ros::Time current_time = transform.header.stamp;
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        double yaw = tf2::getYaw(transform.transform.rotation);

        if (!initialized_) {
            last_x_ = x;
            last_y_ = y;
            last_yaw_ = yaw;
            last_time_ = current_time;
            initialized_ = true;
            return;
        }

        double dt = (current_time - last_time_).toSec();
        if (dt <= 0.0) {
            // 发布零速度
            publishOdom(current_time, x, y, transform.transform.rotation, 0.0, 0.0, 0.0);
            return;
        }

        double dx = x - last_x_;
        double dy = y - last_y_;
        double d_yaw = yaw - last_yaw_;

        // 角度归一化到 [-π, π]
        while (d_yaw > M_PI) d_yaw -= 2.0 * M_PI;
        while (d_yaw < -M_PI) d_yaw += 2.0 * M_PI;

        double vx = dx / dt;
        double vy = dy / dt;
        double vth = d_yaw / dt;

        publishOdom(current_time, x, y, transform.transform.rotation, vx, vy, vth);

        // 更新历史
        last_x_ = x;
        last_y_ = y;
        last_yaw_ = yaw;
        last_time_ = current_time;
    }

private:
    void publishOdom(const ros::Time& stamp,
                     double x, double y,
                     const geometry_msgs::Quaternion& quat,
                     double vx, double vy, double vth) {
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation = quat;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        // 设置协方差（可选，表示高不确定性）
        const double pose_cov = 0.1;
        const double twist_cov = 0.1;
        odom.pose.covariance.assign(0.0);
        odom.twist.covariance.assign(0.0);
        odom.pose.covariance[0]  = pose_cov;   // x
        odom.pose.covariance[7]  = pose_cov;   // y
        odom.pose.covariance[35] = pose_cov;   // yaw
        odom.twist.covariance[0]  = twist_cov; // vx
        odom.twist.covariance[7]  = twist_cov; // vy
        odom.twist.covariance[35] = twist_cov; // vth

        odom_pub_.publish(odom);
    }

    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};
    ros::Publisher odom_pub_;
    ros::Timer timer_;

    std::string base_frame_;
    std::string odom_frame_;
    double publish_rate_;

    bool initialized_ = false;
    double last_x_ = 0.0, last_y_ = 0.0, last_yaw_ = 0.0;
    ros::Time last_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_to_odom");
    TfToOdom node;
    ros::spin();
    return 0;
}