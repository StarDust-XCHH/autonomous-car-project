# Autonomous Racing Robot (STM32 + ROS1)

STM32F446RE + ROS1 racing robot: RPLIDAR C1 + IMU, Bluetooth comms, hector_slam mapping, PID motor control, and a custom move_base local planner optimized for high-speed go-and-return navigation via referee node.

> **Author**: [StarDust 星辰涵], Beijing University of Posts and Telecommunications (BUPT)  
> **License**: MIT (see [LICENSE](LICENSE))

---

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

本项目适用于以下场景：


</details>

<details>
<summary>🇺🇸 English</summary>

This project is suitable for:


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

