// lidar_point_node.cpp
#include <ros/ros.h>
#include <blue_teeth_pkg/RadarPoint.h>
#include <blue_teeth_pkg/RadarPointArray.h>
#include <vector>

// 用于存储雷达点的数组
std::vector<blue_teeth_pkg::RadarPoint> radarPointArray;
// 用于判断是否已经接收到第一个新帧点
bool isFirstPoint = true;
// 用于发布雷达点数组的话题
ros::Publisher array_pub;

/**
 * 雷达点数据回调函数
 * 当有新的雷达点数据发布到/radar/point话题时，此函数会被调用
 */
void radarPointCallback(const blue_teeth_pkg::RadarPoint::ConstPtr& msg)
{
    // 判断是否为新帧的第一个点
    if (msg->is_new_frame) {
        // 如果已经有一个完整的帧数据，则发布它
        if (!isFirstPoint && !radarPointArray.empty()) {
            // 创建雷达点数组消息
            blue_teeth_pkg::RadarPointArray array_msg;
            array_msg.points = radarPointArray;
            
            // 发布雷达点数组
            array_pub.publish(array_msg);
            // ROS_INFO("Published radar point array with %zu points", radarPointArray.size());
        }
        
        // 清空数组并开始新帧
        radarPointArray.clear();
        isFirstPoint = false;
    }
    
    // 如果已经接收到第一个新帧点，则将当前点添加到数组中
    if (!isFirstPoint) {
        // 将当前雷达点添加到数组中
        radarPointArray.push_back(*msg);
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "lidar_point_node");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    
    // 订阅/radar/point话题
    ros::Subscriber sub = nh.subscribe("/radar/point", 10000, radarPointCallback);
    
    // 创建发布雷达点数组的话题
    array_pub = nh.advertise<blue_teeth_pkg::RadarPointArray>("/radar/point_array", 1000);
    
    ROS_INFO("Radar Point Subscriber started, listening to /radar/point");
    
    // 进入循环，等待并处理回调
    ros::spin();
    
    return 0;
}