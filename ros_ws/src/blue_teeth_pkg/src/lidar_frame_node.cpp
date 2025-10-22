#include "ros/ros.h"
#include "blue_teeth_pkg/RadarPointArray.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <limits>
#include <vector>

// 全局Publisher
ros::Publisher laser_scan_pub;

void radarCallback(const blue_teeth_pkg::RadarPointArray::ConstPtr& msg)
{
    // ==================== 设置 LaserScan 消息 ====================
    sensor_msgs::LaserScan scan;
    

    scan.header.frame_id = "laser";  // 坐标系名称

    // 角度范围：-π 到 +π（-180° ~ 180°）
    const double angle_min = -M_PI;
    const double angle_max = M_PI;
    const double angle_increment = M_PI / 180.0; // 1度/步
    const int n_ranges = 360; // 360个角度（-180 ~ 179）

    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = angle_increment;
    scan.scan_time = 0.1;                // 扫描周期：10Hz → 0.1秒
    scan.time_increment = scan.scan_time / 359.0;  // 相邻点之间的时间差
    scan.range_min = 0.02;               // 最小有效距离：3cm
    scan.range_max = 16.0;               // 最大有效距离：16m

    // 初始化所有角度为 inf（表示自由空间）
    scan.ranges.assign(n_ranges, std::numeric_limits<float>::infinity());
    scan.intensities.clear();  // 或者不赋值，默认为空


    // ==================== 角度补偿（倍率1）====================
    // 因为 angle_compensate_multiple = 1，我们只需要将每个点映射到最接近的1度槽位
    // 由于输入已排序，可以直接填充

    for (const auto& point : msg->points)
    {
        // 无效距离：不做障碍物处理，但可保留为 inf（已初始化）
        if (point.distance_mm <= 0) {
            continue;
        }

        double angle_deg = fmod(point.angle_deg, 360.0);
        if (angle_deg < 0) angle_deg += 360.0;

        // 转换为 [-180, 180) 范围，匹配 -π ~ π
        if (angle_deg >= 180.0) {
            angle_deg -= 360.0;
        }

        double angle_rad = angle_deg * M_PI / 180.0;

        // 计算对应索引：[-180, 179] → [0, 359]
        int idx = static_cast<int>(round((angle_rad - angle_min) / angle_increment));
        
        // 边界检查：理论上不会越界，但保险起见
        if (idx < 0) idx = 0;
        if (idx >= n_ranges) idx = n_ranges - 1;

        float distance_m = point.distance_mm / 1000.0f;
        if (distance_m < scan.range_min || distance_m > scan.range_max) {
            continue; // 跳过无效点
        }
        // 保留最近点（可选优化）
        if (std::isinf(scan.ranges[idx]) || distance_m < scan.ranges[idx]) {
            scan.ranges[idx] = distance_m;
            // 如果有强度：scan.intensities[idx] = point.intensity;
        }
    }

    // 使用当前系统时间作为时间戳
    scan.header.stamp = ros::Time::now();

    // ==================== 发布消息 ====================
    laser_scan_pub.publish(scan);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_frame_node");  // 节点名
    ros::NodeHandle nh;

    // 创建发布者
    laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);

    // 订阅雷达点云数组
    ros::Subscriber sub = nh.subscribe("/radar/point_array", 1000, radarCallback);

    ROS_INFO("lidar_frame_node started. Subscribing to /radar/point_array, publishing to /scan");

    ros::spin();  // 进入回调循环
    return 0;
}