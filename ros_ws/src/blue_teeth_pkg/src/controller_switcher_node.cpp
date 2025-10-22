#include <ros/ros.h>
#include <blue_teeth_pkg/SwitchController.h>
#include <iostream>
#include <string>

/**
 * @brief 外部控制器切换节点
 * 功能：从命令行读取用户输入的控制器逻辑名，调用 switch_controller 服务切换当前激活控制器
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_switcher_node");
    ros::NodeHandle nh;

    // 等待服务可用
    ros::ServiceClient client = nh.serviceClient<blue_teeth_pkg::SwitchController>("switch_controller");
    if (!client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("Service 'switch_controller' not available after 5 seconds.");
        return 1;
    }

    std::string input;
    ROS_INFO("=== Controller Switcher ===");
    ROS_INFO("Available controllers (example): 'keyboard', 'dwa'");
    ROS_INFO("Type controller name and press Enter to switch. Type 'quit' to exit.");

    while (ros::ok()) {
        std::cout << "\n> Enter controller name to activate: ";
        std::getline(std::cin, input);

        // 去除首尾空格
        input.erase(0, input.find_first_not_of(" \t\n\r\f\v"));
        input.erase(input.find_last_not_of(" \t\n\r\f\v") + 1);

        if (input == "quit" || input == "exit" || input == "q") {
            ROS_INFO("Exiting controller switcher.");
            break;
        }

        if (input.empty()) {
            ROS_WARN("Input is empty. Please enter a valid controller name.");
            continue;
        }

        // 构造服务请求
        blue_teeth_pkg::SwitchController srv;
        srv.request.target_controller = input;

        // 调用服务
        if (client.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("✅ %s", srv.response.message.c_str());
            } else {
                ROS_WARN("❌ %s", srv.response.message.c_str());
            }
        } else {
            ROS_ERROR("Failed to call service 'switch_controller'. Is the velocity_parser_node running?");
        }
    }

    return 0;
}