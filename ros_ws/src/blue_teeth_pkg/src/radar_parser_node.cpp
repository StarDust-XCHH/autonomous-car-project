// radar_parser_node.cpp

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <blue_teeth_pkg/RadarPoint.h>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <deque>
#include <cctype>

// ==============================
// 自定义结构体
// ==============================
struct LaserPoint {
    int distance_mm;
    double angle_deg;
    bool is_new_frame;

    LaserPoint(int d, double a, bool n) : distance_mm(d), angle_deg(a), is_new_frame(n) {}
};

class LaserPointList {
private:
    static std::vector<LaserPoint> points;
    static double last_angle;
public:
    static void addLaserPoint(const LaserPoint& p) {
        points.push_back(p);
        last_angle = p.angle_deg;
    }
    static double getLastLaserPointAngle() { return last_angle; }
    static const std::vector<LaserPoint>& getPoints() { return points; }
    static void clear() { points.clear(); last_angle = 0.0; }
};
std::vector<LaserPoint> LaserPointList::points;
double LaserPointList::last_angle = 0.0;

// ==============================
// 雷达解析主类（线程安全）
// ==============================
class RadarParserNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::deque<uint8_t> dataBuffer_;

public:
    RadarParserNode() : nh_("~") {
        // 创建发布器
        pub_ = nh_.advertise<blue_teeth_pkg::RadarPoint>("/radar/point", 1000);

        // 订阅蓝牙数据
        sub_ = nh_.subscribe("/bluetooth/received_data", 10000, &RadarParserNode::bluetoothDataCallback, this);

        ROS_INFO("Radar Parser Node started, publishing to /radar/point");
    }

    // 回调函数（运行在ROS内部线程）
    void bluetoothDataCallback(const std_msgs::String::ConstPtr& msg) {
        std::vector<uint8_t> rawData = hexStringToBytes(msg->data);
        if (rawData.empty()) return;

        // 加入缓冲区
        for (uint8_t b : rawData) {
            dataBuffer_.push_back(b);
        }

        // 处理所有完整帧
        while (processNextFrame()) {
            // 继续处理
        }
    }

private:
    // 十六进制字符串转字节
    std::vector<uint8_t> hexStringToBytes(const std::string& hexStr) {
        std::vector<uint8_t> bytes;
        std::stringstream ss(hexStr);
        std::string byteStr;

        while (ss >> byteStr) {
            byteStr.erase(std::remove_if(byteStr.begin(), byteStr.end(), ::isspace), byteStr.end());
            if (byteStr.size() == 0) continue;
            try {
                size_t pos;
                int byte = std::stoi(byteStr, &pos, 16);
                if (pos == byteStr.size()) {
                    bytes.push_back(static_cast<uint8_t>(byte));
                }
            } catch (...) {
                ROS_WARN("Invalid hex byte: %s", byteStr.c_str());
            }
        }
        return bytes;
    }

bool processNextFrame() {
    if (dataBuffer_.size() < 4) return false;

    if (dataBuffer_[0] == 0xFF && dataBuffer_[1] == 0xFF && dataBuffer_[2] == 0xFF) {
        uint8_t payloadLength = dataBuffer_[3];  // ✅ 正确位置
        int totalLen = 4 + payloadLength;

        if (dataBuffer_.size() >= totalLen) {
            std::vector<uint8_t> frame(dataBuffer_.begin(), dataBuffer_.begin() + totalLen);
            dataBuffer_.erase(dataBuffer_.begin(), dataBuffer_.begin() + totalLen); // 更高效

            if (payloadLength == 0x54) {
                // ROS_INFO("ExLidar frame");
                parseExLidarFrame(frame);
            } else if (payloadLength == 0x28) {
                // parseRadarFrame(frame);
            } else if (payloadLength == 0x0C) {
                // parseOdometryFrame(frame);
            } else {
                ROS_WARN("Unkown BT frame");
            }
            return true;
        } else {
            return false;
        }
    } else {
        dataBuffer_.pop_front();
        return true;
    }
}

    void parseExLidarFrame(const std::vector<uint8_t>& frame) {
        int payloadLength = frame[3];
        std::vector<uint8_t> data(payloadLength);
        std::copy(frame.begin() + 4, frame.end(), data.begin());

        int chks1 = data[0] & 0x0F;
        int chks2 = data[1] & 0x0F;
        int checksum = (chks2 << 4) | chks1;

        int xorChecksum = 0;
        for (int i = 2; i < data.size(); ++i) {
            xorChecksum ^= data[i];
        }

        if (xorChecksum != checksum) {
            ROS_ERROR("XOR校验失败");
            return;
        }

        int sync1 = (data[0] & 0xF0) >> 4;
        int sync2 = (data[1] & 0xF0) >> 4;
        if (sync1 != 0xA || sync2 != 0x5) {
            ROS_ERROR("Sync错误: %X %X", sync1, sync2);
            return;
        }

        double startAngle = ((data[3] & 0x7F) << 8) | data[2];
        startAngle /= 64.0;

        int numPoints = (data.size() - 4) / 2;

        for (int i = 4; i < data.size() && i+1 < data.size(); i += 2) {
            int dis = (data[i+1] << 8) | data[i];
            double angle = startAngle + (28.2 / numPoints) * (i/2 - 2);
            if (angle >= 360.0) angle -= 360.0;

            bool isNew = false;
            if (std::abs(angle - LaserPointList::getLastLaserPointAngle()) > 350.0) {
                isNew = true;
            }

            LaserPoint pt(dis, angle, isNew);
            LaserPointList::addLaserPoint(pt);

            // ✅ 安全发布：使用类成员 pub_
            blue_teeth_pkg::RadarPoint msg;
            msg.distance_mm = static_cast<float>(dis);
            msg.angle_deg = static_cast<float>(angle);
            msg.is_new_frame = isNew;

            pub_.publish(msg);  // 线程安全：ROS Publisher 内部有锁
        }
    }
};

// ==============================
// main 函数
// ==============================
int main(int argc, char** argv) {
    ros::init(argc, argv, "radar_parser_node");
    RadarParserNode node;
    ros::spin();
    return 0;
}