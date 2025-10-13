# autonomous-car-project
STM32F446RE+ROS1 racing robot: RPLIDAR C1 + IMU, Bluetooth comms, hector_slam mapping, PID motor control, and a custom move_base local planner optimized for high-speed go-and-return navigation via referee node.

<details open>
<summary>🇨🇳 中文说明</summary>

本项目是一个基于双轮差速驱动（辅以万向轮）的自动驾驶竞速小车，通过蓝牙实现 STM32 嵌入式系统与 ROS1（Noetic）上位机的紧密协同。嵌入式端实时采集 RPLIDAR C1 激光雷达与 IMU 数据，接收上位机下发的期望速度指令，执行高响应 PID 电机控制，并通过蓝牙进行双向通信。上位机端采用 `hector_slam` 实现无里程计的纯激光 SLAM 建图，并对 `move_base` 的局部规划器进行了定制化改造，摒弃传统避障逻辑，专为高速竞速路径跟踪优化。项目还包含一个“裁判节点”，用户只需输入相对于起点的目标坐标，小车即可自动导航至终点并返航。整体设计追求速度、精度与开箱即用的可复现性。
</details>

<details>
<summary>🇺🇸 English</summary>

This project presents an autonomous racing robot based on a differential-drive chassis (with a caster wheel), featuring tight integration between an STM32-based embedded system and a ROS1 (Noetic) navigation stack via Bluetooth. The embedded side handles real-time data acquisition from an RPLIDAR C1 and an IMU, executes PID-controlled motor commands based on velocity targets from the host, and communicates bidirectionally over Bluetooth. On the ROS side, `hector_slam` enables lidar-only SLAM for map building, while a customized local planner in `move_base` is optimized for high-speed racing trajectories—prioritizing path tracking over traditional obstacle avoidance. A dedicated “referee node” allows users to specify a goal coordinate relative to the start point, enabling fully autonomous go-and-return navigation. Designed for speed, precision, and end-to-end reproducibility.
</details>
