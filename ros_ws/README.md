<details open>
<summary>🇨🇳 中文说明</summary>
🐧 ROS1 移植指南（Ubuntu 20.04 + ROS Noetic）

⚠️ 前提要求
本项目依赖 ROS 官方包 hector_slam 和 rviz，请确保已安装 ROS Noetic 桌面完整版（desktop-full）。若未安装，请先参考 [ROS 官方安装指南](http://wiki.ros.org/noetic/Installation/Ubuntu)。

📦 需要移植的 ROS 功能包

本项目 ROS 部分由以下 4 个功能包组成，请勿克隆整个仓库到工作空间，只需复制这些包：

```
blue_teeth_pkg # 蓝牙通信 + 雷达解析 + 控制中枢（含自定义消息）
hector_nav_demo # SLAM 与导航配置（基于 hector_slam）
remoter_pkg # 自定义键盘遥控节点
my_planner # 自定义局部规划器 + 裁判逻辑
```

💡 注意：
blue_teeth_pkg 包含自定义消息类型（如 RadarPoint.msg），
请确保完整复制该包，catkin_make 会自动编译消息并生成头文件。

✅ 移植步骤

1. 创建工作空间（如尚未创建）

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

2. 克隆项目到任意目录（不要在 catkin_ws 内）

```bash
cd ~
git clone https://github.com/StarDust-XCHH/autonomous-car-project.git
```


3. 复制 ROS 功能包到工作空间
```bash
cd ~/catkin_ws/src
cp -r ~/autonomous-car-project/ros_ws/src/blue_teeth_pkg .
cp -r ~/autonomous-car-project/ros_ws/src/hector_nav_demo .
cp -r ~/autonomous-car-project/ros_ws/src/remoter_pkg .
cp -r ~/autonomous-car-project/ros_ws/src/my_planner .
```

4. 安装依赖（通常无需额外操作）
```bash
rosdep install --from-paths . --ignore-src -r -y
```

5. 编译
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

6. 启动系统
```bash
roslaunch blue_teeth_pkg bt_slam.launch
```


</details>

<details>
<summary>🇺🇸 English</summary>
🐧 ROS1 Porting Guide (Ubuntu 20.04 + ROS Noetic)


⚠️ Prerequisite
This project depends on official ROS packages: hector_slam and rviz.
Please ensure you have installed the full desktop version of ROS Noetic.
If not, set up your environment first by following the [Official ROS Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

📦 ROS Packages to Port

The ROS system consists of the following 4 packages. Do NOT clone the entire repository into your workspace—only copy these packages:


```
blue_teeth_pkg # Bluetooth communication + radar parsing + control hub (includes custom messages)
hector_nav_demo # SLAM and navigation config (based on hector_slam)
remoter_pkg # Custom keyboard teleoperation node
my_planner # Custom local planner + referee logic
```

💡 Note:
blue_teeth_pkg contains custom message types (e.g., RadarPoint.msg).
Please copy the entire package—catkin_make will automatically compile the messages and generate headers.

✅ Porting Steps

1. Create a catkin workspace (if not exists)
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

2. Clone the project to any directory (outside the workspace)
```bash
cd ~
git clone https://github.com/yourname/autonomous-car-project.git
```

3. Copy ROS packages into your workspace
```bash
cd ~/catkin_ws/src
cp -r ~/autonomous-car-project/ros_ws/src/blue_teeth_pkg .
cp -r ~/autonomous-car-project/ros_ws/src/hector_nav_demo .
cp -r ~/autonomous-car-project/ros_ws/src/remoter_pkg .
cp -r ~/autonomous-car-project/ros_ws/src/my_planner .
```

4. Install dependencies (usually no extra action needed)
```bash
rosdep install --from-paths . --ignore-src -r -y
```

5. Build the workspace
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

6. Launch the system
```bash
roslaunch blue_teeth_pkg bt_slam.launch
```



</details>

