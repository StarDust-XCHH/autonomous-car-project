#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <cmath>

class VirtualOdom {
private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string odom_frame_;
    std::string base_frame_;
    double publish_rate_;

    ros::Time last_time_;
    geometry_msgs::Point last_position_;
    geometry_msgs::Quaternion last_orientation_;

public:
    VirtualOdom()
        : tf_listener_(tf_buffer_),
          last_time_(ros::Time::now()) {

        // 从参数服务器读取参数
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        nh_.param<double>("rate", publish_rate_, 10.0);

        // 初始化发布器
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);

        ROS_INFO("Virtual Odom (C++) Node Started");
    }

    void spin() {
        ros::Rate rate(publish_rate_);

        while (ros::ok()) {
            ros::Time now = ros::Time::now();
            double dt = (now - last_time_).toSec();

            // 获取 map -> base_link 的最新变换
            geometry_msgs::TransformStamped transform;
            try {
                transform = tf_buffer_.lookupTransform("map", base_frame_, ros::Time(0), ros::Duration(1.0));
            } catch (tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(1, "TF lookup failed: %s", ex.what());
                rate.sleep();
                continue;
            }

            // 提取当前位置和姿态

            geometry_msgs::Point current_position;
            current_position.x = transform.transform.translation.x;
            current_position.y = transform.transform.translation.y;
            current_position.z = transform.transform.translation.z;
            geometry_msgs::Quaternion current_orientation = transform.transform.rotation;

            // 计算速度（第一次跳过）
            geometry_msgs::Twist twist;
            if (last_time_ != ros::Time(0) && dt > 0) {
                twist.linear.x = (current_position.x - last_position_.x) / dt;
                twist.linear.y = (current_position.y - last_position_.y) / dt;

                double current_yaw = quatToYaw(current_orientation);
                double last_yaw = quatToYaw(last_orientation_);
                double dtheta = angleDiff(current_yaw, last_yaw);
                twist.angular.z = dtheta / dt;
            } else {
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.angular.z = 0.0;
            }

            // 创建并发布 Odometry 消息
            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = now;
            odom_msg.header.frame_id = odom_frame_;
            odom_msg.child_frame_id = base_frame_;

            odom_msg.pose.pose.position = current_position;
            odom_msg.pose.pose.orientation = current_orientation;
            odom_msg.twist.twist = twist;

            // 设置协方差（简化）
            for (int i = 0; i < 36; ++i) {
                odom_msg.pose.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
                odom_msg.twist.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
            }

            odom_pub_.publish(odom_msg);

            // 发布 odom -> base_link 的 TF（复制 map->base_link）
            geometry_msgs::TransformStamped odom_tf;
            odom_tf.header.stamp = now;
            odom_tf.header.frame_id = odom_frame_;
            odom_tf.child_frame_id = base_frame_;
            odom_tf.transform = transform.transform;
            tf_broadcaster_.sendTransform(odom_tf);

            // // ✅✅✅ 新增：发布 map -> odom 的单位变换 ✅✅✅
            // geometry_msgs::TransformStamped map_to_odom;
            // map_to_odom.header.stamp = now;
            // map_to_odom.header.frame_id = "map";
            // map_to_odom.child_frame_id = odom_frame_;
            // map_to_odom.transform.translation.x = 0.0;
            // map_to_odom.transform.translation.y = 0.0;
            // map_to_odom.transform.translation.z = 0.0;
            // map_to_odom.transform.rotation.x = 0.0;
            // map_to_odom.transform.rotation.y = 0.0;
            // map_to_odom.transform.rotation.z = 0.0;
            // map_to_odom.transform.rotation.w = 1.0;
            // tf_broadcaster_.sendTransform(map_to_odom);


            // 更新历史状态
            last_time_ = now;
            last_position_ = current_position;
            last_orientation_ = current_orientation;

            rate.sleep();
        }
    }

private:
    double quatToYaw(const geometry_msgs::Quaternion& q) {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    double angleDiff(double a, double b) {
        double diff = a - b;
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        return diff;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "virtual_odom");
    VirtualOdom node;
    node.spin();
    return 0;
}