#include <ros/ros.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <set>
#include <string>
#include <algorithm>
#include <map>
#include <blue_teeth_pkg/SwitchController.h>

class KeyboardListener {
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;  // ROS 发布者，发布按键状态

    std::set<char> currently_pressed_;  // 当前按下的键集合（去重、有序）
    std::set<std::string> valid_states_; // 合法按键状态集合

    ros::ServiceClient switch_controller_client_;
    bool dwa_switch_requested_ = false;  // 防止重复切换（当前未使用，可保留）

    struct termios orig_termios_;  // 原始终端设置，用于程序退出时恢复

    // 静态指针，用于 atexit 回调时访问当前实例
    static KeyboardListener* instance_;

    // 防止重复恢复终端的标志
    bool terminal_restored_ = false;

    // 👇 添加这一行 👇
    std::set<char> last_detected_keys_;  // 上一帧检测到的按键，用于对比释放

    /* ============ 终端设置函数 ============ */

    // 真正的重置函数（私有实现）
    void reset_terminal_mode_impl() {
        if (terminal_restored_) return; // 避免重复恢复
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
        terminal_restored_ = true;
        ROS_INFO("Terminal mode restored.");
    }

    // 静态函数，供 atexit 注册（无 this 指针，符合 C 函数签名）
    static void reset_terminal_mode() {
        if (instance_) {
            instance_->reset_terminal_mode_impl();
        }
    }

    // 设置终端为非阻塞、无回显、非规范模式（立即响应按键）
    void set_conio_terminal_mode() {
        struct termios new_termios;
        tcgetattr(STDIN_FILENO, &orig_termios_);  // 保存原始设置
        new_termios = orig_termios_;

        // 关闭规范模式（行缓冲）和回显
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

        // 设置标准输入为非阻塞
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

        // 注册退出时恢复终端的函数（静态函数）
        atexit(&KeyboardListener::reset_terminal_mode);
    }

    /* ============ 键盘输入检测函数 ============ */

    // 检查是否有键盘输入（非阻塞）
    bool kbhit() {
        struct timeval tv = {0L, 0L};  // 立即返回
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        return select(1, &fds, nullptr, nullptr, &tv) > 0;
    }

    // 读取一个字符（非阻塞）
    char getch() {
        char c = 0;
        ssize_t n = read(STDIN_FILENO, &c, 1);
        return (n > 0) ? c : 0;
    }

    /* ============ 按键状态管理函数 ============ */

    // 生成当前按键组合字符串（按键按字母序排列，如 AW -> "AW"）
    std::string getCurrentStateString() {
        if (currently_pressed_.empty()) {
            return "";
        }

        std::string state(currently_pressed_.begin(), currently_pressed_.end());
        // 排序确保 "WA" 和 "AW" 都变成 "AW"
        std::sort(state.begin(), state.end());
        return state;
    }

    // 检查当前状态是否是合法状态
    bool isValidState(const std::string& state) {
        return valid_states_.find(state) != valid_states_.end();
    }

public:
    /* ============ 构造函数：初始化发布者和合法状态 ============ */
    KeyboardListener() : nh_(), switch_controller_client_(nh_.serviceClient<blue_teeth_pkg::SwitchController>("switch_controller")) {
        instance_ = this;
        pub_ = nh_.advertise<std_msgs::String>("/keyboard_state", 1);
        set_conio_terminal_mode();

        // 等待服务可用（非阻塞，最多等1秒）
        if (!switch_controller_client_.waitForExistence(ros::Duration(1.0))) {
            ROS_WARN("Service 'switch_controller' not available yet. O/K keys will not work until it appears.");
        }

        ROS_INFO("Keyboard listener started. Listening for: W, A, S, D, P, Q, E, O (DWA), K (Keyboard)");
    }

    /* ============ 主循环 ============ */
    void run() {
        fd_set read_fds;
        struct timeval timeout;

        // 👇 新增：记录哪些键已经发布过“按下”消息（直到松开前不再重复发）
        std::set<char> already_published_;

        while (ros::ok()) {
            FD_ZERO(&read_fds);
            FD_SET(STDIN_FILENO, &read_fds);
            timeout.tv_sec = 0;
            timeout.tv_usec = 10000; // 10ms

            int activity = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
            if (activity < 0) {
                ROS_ERROR("select error");
                break;
            }

            std::set<char> detected_this_frame;

            if (activity > 0 && FD_ISSET(STDIN_FILENO, &read_fds)) {
                while (kbhit()) {
                    char c = getch();
                    if (c == 3) { // Ctrl+C
                        ROS_WARN("Ctrl+C detected. Shutting down...");
                        ros::shutdown();
                        return;
                    }

                    if (c >= 'a' && c <= 'z') c -= 32; // 转大写

                    // 👇 注意：O 和 K 也加入检测，但后续不发布
                    if (c == 'W' || c == 'A' || c == 'S' || c == 'D' || 
                        c == 'P' || c == 'Q' || c == 'E' || c == 'O' || c == 'K') {
                        detected_this_frame.insert(c);
                        ROS_DEBUG("Key detected this frame: %c", c);
                    }
                }
            }

            // 👇 找出“刚刚按下”的键：本次检测到，但上次未记录为“已发布”
            std::set<char> newly_pressed;
            for (char c : detected_this_frame) {
                if (already_published_.find(c) == already_published_.end()) {
                    newly_pressed.insert(c);
                }
            }

            // 👇 发布“刚刚按下”的键（但跳过 O 和 K！）
            for (char c : newly_pressed) {
                // ✅ 关键修改：O 和 K 不发布到 /keyboard_state
                if (c == 'O' || c == 'K') {
                    continue;  // 不发布，仅用于切换控制器
                }

                std_msgs::String msg;
                msg.data = std::string(1, c); // 单个字符转字符串
                pub_.publish(msg);
                ROS_DEBUG("Published single key press: %c", c);
                already_published_.insert(c); // 标记为已发布，避免重复
            }

            // 👇 找出“已释放”的键：之前发布过，但本次没检测到
            std::set<char> released_keys;
            std::set_difference(
                already_published_.begin(), already_published_.end(),
                detected_this_frame.begin(), detected_this_frame.end(),
                std::inserter(released_keys, released_keys.begin())
            );

            // 👇 清除已释放键的“已发布”标记（下次再按时可重新发布）
            for (char c : released_keys) {
                already_published_.erase(c);
                ROS_DEBUG("Key released: %c", c);
                // ❗ 不发布任何消息！符合“不按不发布”要求
            }

            // ========== 新增：处理 O 和 K 键的控制器切换逻辑 ==========
            // --- 处理 O：先发 P，再切 DWA ---
            if (newly_pressed.find('O') != newly_pressed.end()) {
                // 发布 P 停止
                std_msgs::String p_msg;
                p_msg.data = "P";
                pub_.publish(p_msg);
                ROS_DEBUG("Published 'P' to stop robot before switching to DWA.");

                // 等待 50ms
                ros::Duration(0.05).sleep();

                // 切换控制器
                blue_teeth_pkg::SwitchController srv;
                srv.request.target_controller = "dwa";

                ROS_DEBUG("Calling switch_controller service to switch to DWA...");
                if (switch_controller_client_.call(srv)) {
                    if (srv.response.success) {
                        ROS_WARN("✅ Successfully switched to DWA controller.");
                    } else {
                        ROS_WARN("❌ Failed to switch to DWA controller: %s", 
                                srv.response.message.c_str());
                    }
                } else {
                    ROS_WARN("❌ Failed to call switch_controller service for DWA. Is the service running?");
                }
            }

            if (newly_pressed.find('K') != newly_pressed.end()) {
                blue_teeth_pkg::SwitchController srv;
                srv.request.target_controller = "keyboard";  // ✅ 字段名一致

                ROS_DEBUG("Calling switch_controller service to switch to Keyboard control...");

                if (switch_controller_client_.call(srv)) {
                    if (srv.response.success) {
                        ROS_WARN("✅ Successfully switched to Keyboard controller.");
                    } else {
                        ROS_WARN("❌ Failed to switch to Keyboard controller: %s", 
                                 srv.response.message.c_str());
                    }
                } else {
                    ROS_WARN("❌ Failed to call switch_controller service for Keyboard. Is the service running?");
                }
            }
            // ========== 控制器切换逻辑结束 ==========

            ros::spinOnce();
        }
    }

    /* ============ 析构函数 ============ */
    ~KeyboardListener() {
        // 清空实例指针，防止 atexit 回调时访问已销毁对象
        instance_ = nullptr;
        // 确保终端恢复（即使 atexit 已调用，重复调用也没关系，因为有保护标志）
        reset_terminal_mode_impl();
    }
};

// 初始化静态成员变量
KeyboardListener* KeyboardListener::instance_ = nullptr;

/* ============ 主函数 ============ */
int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_listener_node");
    ROS_INFO("Starting keyboard listener node...");

    KeyboardListener kl;
    kl.run();

    return 0;
}