#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>

class BluetoothNode {
private:
    ros::NodeHandle nh_;
    ros::Publisher data_pub_;
    ros::Subscriber cmd_sub_;
    std::unique_ptr<serial::Serial> ser_;
    std::thread read_thread_;
    std::thread write_thread_;
    std::queue<std::string> write_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cond_;
    volatile bool running_;

    // 十六进制转字符串（可选格式化）
    std::string bytesToHex(const uint8_t* data, size_t length) {
        std::stringstream ss;
        ss << std::hex << std::setfill('0');
        for (size_t i = 0; i < length; ++i) {
            ss << std::setw(2) << static_cast<int>(data[i]);
            if (i < length - 1) ss << " ";
        }
        return ss.str();
    }

    // 串口读取线程（关键：低延迟）
    void readSerial() {
        ROS_INFO("Starting serial read thread...");
        uint8_t buffer[1024];
        while (running_ && ros::ok()) {
            try {
                size_t n = ser_->read(buffer, sizeof(buffer));
                if (n > 0) {
                    std::string hex_data = bytesToHex(buffer, n);
                    std_msgs::String msg;
                    msg.data = hex_data;
                    data_pub_.publish(msg);
                    ros::spinOnce(); // 及时处理回调
                }
            } catch (const serial::IOException& e) {
                ROS_ERROR_STREAM("Read error: " << e.what());
                running_ = false;
                break;
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("Unexpected read error: " << e.what());
            }
        }
    }

    // 串口写入线程（避免阻塞主线程）
    void writeSerial() {
        ROS_INFO("Starting serial write thread...");
        while (running_ && ros::ok()) {
            std::string data;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cond_.wait(lock, [this] { return !write_queue_.empty() || !running_; });
                if (!running_) break;
                data = write_queue_.front();
                write_queue_.pop();
            }

            try {
                // ✅ 直接以字符串形式写入（ASCII 字节流）
                ser_->write(data);  // ← 直接写入字符串

                // 可选：自动添加换行符（如果设备需要）
                // ser_->write("\r\n");  // 根据设备协议选择 \n, \r, 或 \r\n

                ROS_INFO("Sent (as string): %s", data.c_str());
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("Write error: " << e.what());
            }
        }
    }

    // 订阅回调：接收命令
    void commandCallback(const std_msgs::String::ConstPtr& msg) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            write_queue_.push(msg->data);
        }
        queue_cond_.notify_one();
    }

public:
    BluetoothNode() : running_(true) {
        // 初始化串口
        ser_ = std::make_unique<serial::Serial>(
            "/dev/rfcomm0",      // 设备路径
            921600,              // 波特率
            serial::Timeout::simpleTimeout(10) // 10ms 超时
        );

        if (!ser_->isOpen()) {
            ROS_FATAL("Failed to open serial port.");
            running_ = false;
            return;
        }

        ROS_INFO("Serial port opened successfully.");

        // 创建发布者和订阅者
        data_pub_ = nh_.advertise<std_msgs::String>("/bluetooth/received_data", 1000);
        cmd_sub_ = nh_.subscribe("/bluetooth/send_command", 100, &BluetoothNode::commandCallback, this);

        // 启动读写线程
        read_thread_ = std::thread(&BluetoothNode::readSerial, this);
        write_thread_ = std::thread(&BluetoothNode::writeSerial, this);
    }

    ~BluetoothNode() {
        running_ = false;
        queue_cond_.notify_all();
        if (read_thread_.joinable()) read_thread_.join();
        if (write_thread_.joinable()) write_thread_.join();
        if (ser_->isOpen()) ser_->close();
    }

    void spin() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bluetooth_node");
    ROS_INFO("Bluetooth node starting...");

    BluetoothNode node;
    node.spin();

    return 0;
}