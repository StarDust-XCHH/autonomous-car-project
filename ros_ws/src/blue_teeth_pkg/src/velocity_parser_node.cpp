#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <blue_teeth_pkg/SwitchController.h>
#include <mutex>
#include <map>
#include <XmlRpcValue.h>  // For parsing parameter server mapping
#include <std_msgs/String.h>
#include <cmath>

/**
 * @brief Velocity Parser Class
 * Features:
 *   - Subscribes to /cmd_vel to receive velocity commands
 *   - Only processes messages from the publisher node mapped to the "currently active controller (logical name)"
 *   - Provides switch_controller service to dynamically switch active controller (by logical name)
 *   - Converts linear & angular velocity to left/right wheel speeds (differential drive model)
 */
class VelocityParser {
public:
    /**
     * @brief Constructor: Initialize parameters, subscriber, service, controller mapping
     */
    VelocityParser() {
        // Load wheel base from parameter server (default: 0.165m)
        nh_.param<double>("wheel_base", wheel_base_, 0.165);

        // Default active controller (logical name)
        std::string default_controller = "dwa";
        nh_.param<std::string>("active_controller", active_controller_, default_controller);

        // Load controller mapping: logical name → actual node name
        XmlRpc::XmlRpcValue mapping_param;
        if (nh_.getParam("controller_mapping", mapping_param) && 
            mapping_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            for (auto& entry : mapping_param) {
                std::string logic_name = entry.first;
                std::string real_name = static_cast<std::string>(entry.second);
                controller_mapping_[logic_name] = real_name;
                ROS_INFO("Loaded controller mapping: [%s] → %s", logic_name.c_str(), real_name.c_str());
            }
        } else {
            ROS_WARN("controller_mapping parameter not found, using defaults");
            // Set default mappings (for debugging)s
            controller_mapping_["keyboard"] = "/keyboard_subscriber_node";
            controller_mapping_["dwa"] = "/move_base";
        }

        ROS_INFO("Active controller (logical name): %s", active_controller_.c_str());

        // Subscribe to /cmd_vel
        sub_ = nh_.subscribe(
            "/cmd_vel", 
            1,
            &VelocityParser::cmdVelCallback,
            this,
            ros::TransportHints().tcpNoDelay()
        );

        // Advertise switch_controller service
        switch_controller_srv_ = nh_.advertiseService(
            "switch_controller", 
            &VelocityParser::switchControllerCallback, 
            this
        );

        // ✅ Initialize Bluetooth command publisher
        bluetooth_pub_ = nh_.advertise<std_msgs::String>("/bluetooth/send_command", 10);

        ROS_INFO("VelocityParser node started. Service 'switch_controller' is ready.");
        ROS_INFO("Subscribed to /cmd_vel. Wheel base: %.3f m", wheel_base_);
    }

private:
    const double WHEEL_CIRCUMFERENCE = 0.20420335; // meters
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::ServiceServer switch_controller_srv_;
    double wheel_base_;
    std::string active_controller_;        // Currently active controller (logical name)
    std::map<std::string, std::string> controller_mapping_; // logical → actual node name
    std::mutex controller_mutex_;
    ros::Publisher bluetooth_pub_;  // Publishes encoded wheel speed strings

    /**
     * @brief Callback for velocity commands
     * Processes message only if publisher matches currently active controller's mapped node name
     */
    void cmdVelCallback(const ros::MessageEvent<geometry_msgs::Twist>& event) {
        const geometry_msgs::Twist::ConstPtr& msg = event.getMessage();
        if (!msg) {
            ROS_WARN("Received empty message!");
            return;
        }

        std::string publisher_name = event.getPublisherName();
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        ROS_DEBUG("Received cmd_vel from node: %s", publisher_name.c_str());

        std::string current_active_logic;
        std::string current_active_real;

        {
            std::lock_guard<std::mutex> lock(controller_mutex_);
            current_active_logic = active_controller_;

            auto it = controller_mapping_.find(current_active_logic);
            if (it == controller_mapping_.end()) {
                ROS_ERROR("No actual node name found for logical controller: '%s'", current_active_logic.c_str());
                return;
            }
            current_active_real = it->second;
        }

        if (publisher_name == current_active_real) {
            ROS_INFO("[%s] Active controller processing command.", current_active_logic.c_str());

            // ✅ Special handling for DWA: invert velocities due to base_link orientation mismatch
            if (current_active_logic == "dwa") {
                linear_x = linear_x;
                angular_z = -angular_z;
                ROS_WARN_THROTTLE(1.0, "DWA velocity inverted: linear_x=%.3f, angular_z=%.3f", linear_x, angular_z);
            }
            
            processVelocityCommand(linear_x, angular_z);
        } else {
            ROS_DEBUG("Ignoring command from non-active controller: %s", publisher_name.c_str());
        }
    }

    /**
     * @brief Execute velocity command: convert to left/right wheel speeds and publish as string
     */
    void processVelocityCommand(double linear_x, double angular_z) {


        // // 死区补偿
        // if(linear_x == 0.0){
        //     linear_x = 0.0;
        // }else if (linear_x>0.0001&&linear_x<0.10)
        // {
        //     /* code */
        //     linear_x = 0.10;
        // }else if (linear_x<-0.0001&&linear_x>-0.10)
        // {
        //     /* code */
        //     linear_x = -0.10;
        // }
        
        



        double v_left  = linear_x - (angular_z * wheel_base_) / 2.0;
        double v_right = linear_x + (angular_z * wheel_base_) / 2.0;

        // ROS_INFO("Executing: Left wheel: %.3f m/s, Right wheel: %.3f m/s", v_left, v_right);

        // Encode as string: "1 <left_rpm> <right_rpm>"
        std_msgs::String msg;
        char buffer[64];

        double rotational_speed_L = v_left * 60 / WHEEL_CIRCUMFERENCE;
        double rotational_speed_R = v_right * 60 / WHEEL_CIRCUMFERENCE;

        int rpm_L = static_cast<int>(std::round(rotational_speed_L));
        int rpm_R = static_cast<int>(std::round(rotational_speed_R));

        snprintf(buffer, sizeof(buffer), "1 %d %d", rpm_L, rpm_R);
        msg.data = std::string(buffer);

        bluetooth_pub_.publish(msg);

        ROS_DEBUG("Published Bluetooth command: %s", msg.data.c_str());
    }

    /**
     * @brief Service callback to switch active controller (by logical name)
     */
    bool switchControllerCallback(blue_teeth_pkg::SwitchController::Request &req,
                                  blue_teeth_pkg::SwitchController::Response &res) {
        std::lock_guard<std::mutex> lock(controller_mutex_);
        std::string target_logic = req.target_controller;

        if (target_logic.empty()) {
            res.success = false;
            res.message = "Controller logical name cannot be empty";
            ROS_WARN("Switch rejected: %s", res.message.c_str());
            return true;
        }

        if (controller_mapping_.find(target_logic) == controller_mapping_.end()) {
            res.success = false;
            res.message = "Unknown controller logical name: " + target_logic;
            ROS_WARN("Switch rejected: %s", res.message.c_str());
            return true;
        }

        std::string old_controller = active_controller_;
        active_controller_ = target_logic;
        res.success = true;
        res.message = "Controller switched from [" + old_controller + "] to [" + target_logic + "]";
        // ROS_INFO("%s", res.message.c_str());
        return true;
    }
};

/**
 * @brief Main function
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_parser_node");
    VelocityParser parser;
    ros::spin();
    return 0;
}