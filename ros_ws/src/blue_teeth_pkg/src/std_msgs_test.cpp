#include <ros/ros.h>
#include <std_msgs/String.h>  // 测试 std_msgs 是否可用

int main(int argc, char** argv) {
    ros::init(argc, argv, "std_msgs_test");
    ros::NodeHandle nh;

    // 创建一个发布者，发布到 /test_string 话题
    ros::Publisher pub = nh.advertise<std_msgs::String>("/test_string", 10);

    ros::Rate rate(1); // 1 Hz

    ROS_INFO("std_msgs test node started. Publishing to /test_string");

    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hello from std_msgs test";

        pub.publish(msg);
        ROS_INFO_STREAM("Published: " << msg.data);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}