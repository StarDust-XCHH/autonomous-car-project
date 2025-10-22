#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "bluetooth_send_interactive");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("/bluetooth/send_command", 10);
    ros::Rate rate(10); // 控制最大发送频率

    ROS_INFO("Bluetooth Interactive Sender Ready.");
    ROS_INFO("Type a message and press Enter to send.");
    ROS_INFO("Type 'quit' or 'exit' to exit.");

    std::string line;
    while (ros::ok()) {
        std::cout << "> ";
        std::getline(std::cin, line);

        if (line.empty()) continue;

        if (line == "quit" || line == "exit") {
            ROS_INFO("Exiting...");
            break;
        }

        std_msgs::String msg;
        msg.data = line;
        pub.publish(msg);
        ROS_INFO("Sent: %s", line.c_str());

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}