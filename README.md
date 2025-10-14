# Autonomous Racing Robot (STM32 + ROS1)

STM32F446RE + ROS1 racing robot: RPLIDAR C1 + IMU, Bluetooth comms, hector_slam mapping, PID motor control, and a custom move_base local planner optimized for high-speed go-and-return navigation via referee node.

> **Author**: [StarDust 星辰涵], Beijing University of Posts and Telecommunications (BUPT)  
> **License**: MIT (see [LICENSE](LICENSE))

## 📑 目录 / Table of Contents

- [Autonomous Racing Robot (STM32 + ROS1)](#autonomous-racing-robot-stm32--ros1)
- [📄 项目说明 / Project Description](#-项目说明--project-description)
- [🎥 演示视频 / Demo Video](#-演示视频--demo-video)
- [🛠 适用场景 / Use Cases](#-适用场景--use-cases)
- [🔧 消息通信架构 / Communication Architecture](#-消息通信架构--communication-architecture)
- [📂 项目结构 / Project Structure](#-项目结构--project-structure)
- [⚙️ 关键模块说明 / Key Modules Overview](#️-关键模块说明--key-modules-overview)
- [🔁 移植建议 / Porting Guide](#-移植建议--porting-guide)
- [🐞 已知问题 / Known Issues](#-已知问题--known-issues)

---

## 📄 项目说明 / Project Description

<details open>
<summary>🇨🇳 中文说明</summary>

本项目是一个基于双轮差速驱动（辅以万向轮）的自动驾驶竞速小车，通过蓝牙实现 STM32 嵌入式系统与 ROS1（Noetic）上位机的紧密协同。嵌入式端实时采集 RPLIDAR C1 激光雷达与 IMU 数据，接收上位机下发的期望速度指令，执行高响应 PID 电机控制，并通过蓝牙进行双向通信。上位机端采用 `hectorMapping_slam` 实现无里程计的纯激光 SLAM 建图，并对 `move_base` 的局部规划器进行了定制化改造，摒弃传统避障逻辑，专为高速竞速路径跟踪优化。项目还包含一个“裁判节点”，用户只需输入相对于起点的目标坐标，小车即可自动导航至终点并返航。整体设计追求远程透传的slam建图、竞速和精准控制。
</details>

<details>
<summary>🇺🇸 English</summary>

This project presents an autonomous racing robot based on a differential-drive chassis (with a caster wheel), featuring tight integration between an STM32-based embedded system and a ROS1 (Noetic) navigation stack via Bluetooth. The embedded side handles real-time data acquisition from an RPLIDAR C1 and an IMU, executes PID-controlled motor commands based on velocity targets from the host, and communicates bidirectionally over Bluetooth. On the ROS side, `hectorMapping_slam` enables lidar-only SLAM for map building, while a customized local planner in `move_base` is optimized for high-speed racing trajectories—prioritizing path tracking over traditional obstacle avoidance. A dedicated “referee node” allows users to specify a goal coordinate relative to the start point, enabling fully autonomous go-and-return navigation. The overall design aims for transparent SLAM mapping, racing-oriented planning, and precise control.
</details>

---

## 🎥 演示视频 / Demo Video

<details open>
<summary>🇨🇳 中文</summary>

- **完整演示（Bilibili）**: [点击观看视频](https://www.bilibili.com/video/BVxxxxxx)  
- **快速预览（GIF）**:

![Demo GIF](assets/demo.gif)
</details>

<details>
<summary>🇺🇸 English</summary>

- **Full Demo (Bilibili)**: [Watch on Bilibili](https://www.bilibili.com/video/BVxxxxxx)  
- **Quick Preview (GIF)**:

![Demo GIF](assets/demo.gif)
</details>


---

## 🛠 适用场景 / Use Cases

<details open>
<summary>🇨🇳 中文</summary>

本项目遵循**最小化与轻量化设计原则**，代码结构精简，专注于在资源受限条件下完成特定任务。其典型适用场景包括：

- **未知静态迷宫中的自主探索与竞速**：用户只需设定一个相对于起点的目标坐标，小车即可在**无预载地图**的情况下，边通过 `hector_slam` 实时建图，边规划路径前往终点并自动返航。整个过程无需人工干预，适用于封闭、静态但初始未知的赛道环境。
- **远程透传式 SLAM 架构参考**：当激光雷达（如 RPLIDAR C1）部署在嵌入式端（STM32），而 SLAM 与导航计算需在远程 ROS1 主机完成时，本项目提供了一套基于蓝牙的低延迟、双向透传通信方案，可作为**嵌入式传感器 + 远程计算平台**协同系统的参考实现。
- **教学或竞赛原型开发**：适合用于机器人学、嵌入式系统与 ROS 集成的教学演示，或作为“起点-目标-返航”类竞速任务的快速原型基础。

> ⚠️ **注意事项**：  
> 本项目**未实现动态障碍物避障**——为提升竞速性能，局部规划器已移除代价地图的实时更新与滤波机制，因此仅适用于**静态环境**（无移动障碍物）。  
> 若无需蓝牙透传、可直接通过串口连接雷达与主机，推荐使用官方 [rplidar_ros](https://github.com/Slamtec/rplidar_ros) 驱动，其稳定性与兼容性更佳。

</details>

<details>
<summary>🇺🇸 English</summary>

This project follows a **minimalist and lightweight design philosophy**, with streamlined code focused on accomplishing specific tasks under resource constraints. It is best suited for the following scenarios:

- **Autonomous exploration and racing in unknown static mazes**: Users only need to specify a goal coordinate relative to the starting point. The robot will then **autonomously explore, build a map in real time using `hector_slam`**, navigate to the goal, and return—**without requiring a pre-loaded map**. This makes it ideal for closed, static environments that are initially unknown.
- **Reference implementation for remote SLAM via transparent transmission**: When the LiDAR (e.g., RPLIDAR C1) is mounted on an embedded platform (STM32) while SLAM and navigation run on a remote ROS1 host, this project provides a low-latency, bidirectional Bluetooth-based transparent communication framework. It serves as a practical reference for **embedded sensor + remote compute** architectures.
- **Educational or competition prototyping**: Useful for teaching ROS-embedded integration, SLAM, and autonomous navigation, or as a rapid prototype for “go-to-goal-and-return” robotics challenges.

> ⚠️ **Note**:  
> This project **does not support dynamic obstacle avoidance**. To maximize racing performance, the local planner disables real-time costmap updates and filtering. Therefore, it is **only suitable for static environments** (no moving obstacles).  
> If a direct serial connection between LiDAR and host is feasible (i.e., no Bluetooth relay needed), the official [rplidar_ros](https://github.com/Slamtec/rplidar_ros) driver is a more robust and maintainable choice.

</details>

---

## 🔧 消息通信架构 / Communication Architecture

<details open>
<summary>🇨🇳 中文</summary>

本项目构建了一个跨平台的**嵌入式-上位机协同通信架构**，通过蓝牙实现 STM32 嵌入式系统与 ROS1 主机之间的双向数据透传。下图为整个系统的消息流结构示意图，包含 ROS 节点间通信（`rqt_graph` 风格）与嵌入式端控制逻辑的融合视图。

![消息通信架构图](https://your-repo.com/path/to/communication_diagram.png)  
*图：系统整体消息流与模块交互关系*

### 📡 通信流程解析：

1. **传感器数据上传**：
   - RPLIDAR C1 的原始激光数据（十六进制）由 `btYawF2H` 模块解析后，通过蓝牙发送至 ROS 端；
   - IMU 数据经 `imu_yaw` 处理后，作为姿态信息同步上传；
   - 所有传感器数据在 ROS 端由 `/radar_parser_node` 解析并发布到 `/scan` 主题，供 SLAM 使用。

2. **导航指令下发**：
   - 用户设定目标点后，`move_base` 输出期望速度指令 `/cmd_vel`；
   - 该指令经 `/velocity_parser_node` 转换为蓝牙可传输格式（如 `BT_yaw`, `BT_rmv`, `BT_lrv`），通过蓝牙发送至 STM32；
   - STM32 接收后，结合编码器反馈，执行 PID 控制，驱动电机。

3. **闭环反馈机制**：
   - STM32 实时将电机实际速度、IMU 角度等状态通过蓝牙回传；
   - ROS 端接收后更新里程计（`odom`）和状态估计，形成闭环控制。

4. **特殊优化设计**：
   - 为提升竞速性能，局部规划器已移除代价地图更新逻辑，仅保留路径跟踪；
   - `hector_slam` 无需里程计，直接基于激光建图，适用于无轮式编码器的场景。

> ✅ 本架构实现了“**传感器数据上行 + 控制指令下行**”的轻量化透传模式，适合资源受限但需远程 SLAM 的应用。

</details>

<details>
<summary>🇺🇸 English</summary>

This project establishes a cross-platform communication architecture between the embedded system (STM32) and the ROS1 host, enabling bidirectional data transmission via Bluetooth. The following diagram illustrates the complete message flow, combining ROS node interactions (`rqt_graph` style) with the embedded control logic.

![Communication Architecture Diagram](https://your-repo.com/path/to/communication_diagram.png)  
*Figure: System-wide message flow and module interaction*

### 📡 Communication Flow Breakdown:

1. **Sensor Data Upload**:
   - Raw laser data from RPLIDAR C1 (in hexadecimal format) is parsed by `btYawF2H` and transmitted over Bluetooth to the ROS side;
   - IMU orientation data is processed by `imu_yaw` and uploaded in real time;
   - All sensor data is parsed by `/radar_parser_node` on the ROS side and published to `/scan`, feeding into `hector_slam`.

2. **Navigation Command Downlink**:
   - After a goal is set, `move_base` generates target velocity commands via `/cmd_vel`;
   - These are converted by `/velocity_parser_node` into Bluetooth-compatible format (e.g., `BT_yaw`, `BT_rmv`, `BT_lrv`) and sent to STM32;
   - STM32 receives the command, applies PID control based on encoder feedback, and drives the motors.

3. **Closed-Loop Feedback**:
   - STM32 continuously sends back motor speed, IMU yaw, and other status data via Bluetooth;
   - ROS receives this feedback to update odometry (`odom`) and state estimation, forming a closed-loop control system.

4. **Key Design Optimizations**:
   - To enhance racing performance, the local planner disables costmap updates and filtering, focusing solely on path tracking;
   - `hector_slam` operates without odometry, relying purely on laser data—ideal for systems without wheel encoders.

> ✅ This architecture enables a lightweight transparent transmission model: **sensor data up, control commands down**, suitable for resource-constrained systems requiring remote SLAM.

</details>

---





## 📂 项目结构 / Project Structure

<details open>
<summary>🇨🇳 中文</summary>

```text
autonomous-car-project/
├── .gitignore
├── README.md
├── LICENSE
├── embedded/               # STM32CubeMX + Keil MDK 项目
│   ├── APP/
│   ├── Core/
│   ├── Drivers/
│   ├── MDK-ARM/
│   └── MPU6050.ioc               # CubeMX 配置文件
├── ros_ws/
│   └── src/
│       ├── blue_teeth_pkg  # 蓝牙通信 + 雷达解析 + 控制中枢
│       ├── hector_nav_demo # SLAM + 导航 + 自定义规划器
│       └── remoter_pkg     # 自定义键盘遥控
└── assets/                 # 图片、GIF 等资源
```

</details>

<details>
<summary>🇺🇸 English</summary>

```text
autonomous-car-project/
├── .gitignore
├── README.md
├── LICENSE
├── embedded/               # STM32CubeMX + Keil MDK project
│   ├── APP/
│   ├── Core/
│   ├── Drivers/
│   ├── MDK-ARM/
│   └── MPU6050.ioc               # CubeMX config
├── ros_ws/
│   └── src/
│       ├── blue_teeth_pkg  # Bluetooth + radar parsing + control hub
│       ├── hector_nav_demo # SLAM + navigation + custom planner
│       └── remoter_pkg     # Custom keyboard teleoperation
└── assets/                 # Images, GIFs
```

</details>

---

## ⚙️ 关键模块说明 / Key Modules Overview

<details open>
<summary>🇨🇳 中文</summary>

### 嵌入式端（STM32F446RE）


### ROS1 上位机

</details>

<details>
<summary>🇺🇸 English</summary>

### Embedded Side (STM32F446RE)


### ROS1 Host

</details>

---

## 🔁 移植建议 / Porting Guide

<details open>
<summary>🇨🇳 中文</summary>


</details>

<details>
<summary>🇺🇸 English</summary>

</details>

---

## 🐞 已知问题 / Known Issues

<details open>
<summary>🇨🇳 中文</summary>


</details>

<details>
<summary>🇺🇸 English</summary>


</details>

---

