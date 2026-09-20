# hex_ros_demo_arm_follow — Hello Y6 主从跟随演示

**中文** | [English](README.md)

## 目录

- [项目概述](#项目概述)
- [快速使用](#快速使用)
- [安装](#安装)
- [话题接口](#话题接口)
- [参数](#参数)
- [项目结构](#项目结构)

## 项目概述

`hex_ros_demo_arm_follow` 是 Hello Y6 到 Archer Y6 / Firefly Y6 的主从跟随演示包。操作者拖动 Hello Y6 提供主臂状态，从臂接收控制指令并同步机械臂关节位置和夹爪动作。

主要提供：

- Hello Y6 真机到 Archer Y6 仿真的跟随；
- 单组或双组 Hello Y6 到真实机械臂的跟随；
- 可独立集成的跟随控制节点。

本包支持 **ROS 2 Humble**，兼容 **ROS 1 Noetic**。

## 快速使用

> 请先完成[安装](#安装)，再选择以下启动入口。

### 1. 启动场景

#### 真机对仿真（hello2sim）

该场景连接 Hello Y6 真机主臂和 Archer Y6 仿真从臂，用于验证跟随逻辑。

ROS 2：

```shell
ros2 launch hex_ros_demo_arm_follow hello2sim_follow.launch.py \
    master_robot_host:=192.168.1.100 master_robot_port:=8439
```

ROS 1：

```shell
roslaunch hex_ros_demo_arm_follow hello2sim_follow.launch \
    master_robot_host:=192.168.1.100 master_robot_port:=8439
```

#### 真机对真机单组（hello2real）

主臂为 Hello Y6 真机，从臂可为 Archer Y6 或 Firefly Y6 真机。

ROS 2：

```shell
ros2 launch hex_ros_demo_arm_follow hello2real_follow.launch.py \
    master_robot_host:=192.168.1.100 master_robot_port:=8439 \
    slave_robot_host:=192.168.1.101 slave_robot_port:=8439 \
    robot_type:=archer robot_grip_type:=empty enable_keyboard:=true
```

ROS 1：

```shell
roslaunch hex_ros_demo_arm_follow hello2real_follow.launch \
    master_robot_host:=192.168.1.100 master_robot_port:=8439 \
    slave_robot_host:=192.168.1.101 slave_robot_port:=8439 \
    robot_type:=archer robot_grip_type:=empty enable_keyboard:=true
```

#### 双组真机对真机（dual_hello2real）

两组主从臂同时运行，左侧和右侧各一组。

ROS 2：

```shell
ros2 launch hex_ros_demo_arm_follow dual_hello2real_follow.launch.py \
    left_master_robot_host:=192.168.1.100 left_master_robot_port:=8439 \
    left_slave_robot_host:=192.168.1.101 left_slave_robot_port:=8439 \
    left_robot_type:=archer left_robot_grip_type:=empty \
    right_master_robot_host:=192.168.1.100 right_master_robot_port:=9439 \
    right_slave_robot_host:=192.168.1.101 right_slave_robot_port:=9439 \
    right_robot_type:=archer right_robot_grip_type:=empty enable_keyboard:=true
```

ROS 1：

```shell
roslaunch hex_ros_demo_arm_follow dual_hello2real_follow.launch \
    left_master_robot_host:=192.168.1.100 left_master_robot_port:=8439 \
    left_slave_robot_host:=192.168.1.101 left_slave_robot_port:=8439 \
    left_robot_type:=archer left_robot_grip_type:=empty \
    right_master_robot_host:=192.168.1.100 right_master_robot_port:=9439 \
    right_slave_robot_host:=192.168.1.101 right_slave_robot_port:=9439 \
    right_robot_type:=archer right_robot_grip_type:=empty enable_keyboard:=true
```

### 2. 键盘控制

- 从臂上线完成后自动开始跟随，无需按键。
- 按 **`q`** 停止跟随控制，从臂归位后退出。

双臂 launch 中，`enable_keyboard` 默认控制一个位于根命名空间的公共键盘节点：

- 设置为 `false` 时不启动键盘节点。
- 左右 follow 都使用 `/teleop_keyboard_state`。

> 双臂 launch 使用 `/left/master/*`、`/left/slave/*`、`/right/master/*` 和 `/right/slave/*`；左右 follow 节点共享 `/teleop_keyboard_state`。

## 安装

### 前置条件

- 已安装 **ROS 2 Humble**；使用 ROS 1 时安装 **ROS 1 Noetic**。
- 已安装 Python 3、`pip3`、Git，以及所选 ROS 版本的构建工具。
- 真机场景需确保设备网络可达，并准备好实际 IP、端口和设备型号。

### 1. 安装 Python 依赖

```shell
pip3 install \
    'hex-util-msg>=0.1.0' \
    'hex-util-ros>=0.1.0' \
    'hex-driver-robot>=0.1.0'
```

### 2. 创建并进入工作空间

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 3. 克隆 ROS 包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_follow.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

### 4. 编译包

**ROS 2：**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

**ROS 1：**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

## 话题接口


| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `slave/manip_ctrl` | `hex_ros_msgs/msg/HexRosRoboManipCtrlStamped` | 从臂控制消息 |
| 发布 | `master/color_cmd` | `std_msgs/msg/ColorRGBA` | 主臂颜色消息 |
| 订阅 | `master/manip_state` | `hex_ros_msgs/msg/HexRosRoboManipStateStamped` | 主臂状态消息 |
| 订阅 | `slave/manip_state` | `hex_ros_msgs/msg/HexRosRoboManipStateStamped` | 从臂状态消息 |
| 订阅 | `master/joy_state` | `hex_ros_msgs/msg/HexRosTeleopHandleStateStamped` | 主臂手柄状态消息 |
| 订阅 | `teleop_keyboard_state` | `hex_ros_msgs/msg/HexRosTeleopKeyboardStateStamped` | 键盘状态消息 |

> Hello Y6 主臂只提供状态与手柄输入；控制指令发布到从臂。
>
> 消息类型描述：[hex_ros_msgs public APIs](https://github.com/hexfellow/hex_ros_msgs#public-apis)

## 参数

### 启动参数

| 参数 | 含义 / 取值 |
|---|---|
| `master_robot_host / master_robot_port` | 主臂 IP / 端口 |
| `slave_robot_host / slave_robot_port` | 从臂 IP / 端口；仅完整真机场景 |
| `robot_type` | `archer` 或 `firefly` |
| `robot_grip_type` | gp100 / gp80 / gr100 / empty |
| `viewer / rviz` | 仿真窗口开关；仅仿真场景 |
| `enable_keyboard` | 完整真机场景键盘开关；双臂连接参数加 left_ / right_ 前缀 |

### 节点参数

参数在 `config/ros1/arm_follow.yaml` 和 `config/ros2/arm_follow.yaml` 中设置，ROS 1 与 ROS 2 的默认值一致。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rate_ros` | `1000.0` | 跟随控制循环频率 [Hz] |
| `rate_teleop` | `100.0` | 键盘监听频率 [Hz] |
| `model_urdf` | `""` | URDF 模型文件路径（launch 自动设置，节点未使用） |
| `model_frame_id` | `base_link` | 机器人基坐标系 |
| `gravity` | `[0.0, 0.0, -9.81]` | 重力加速度向量 [m/s²] |
| `arm_end_pos` | `[0.0, -1.5, 3.0, 0.07, 0.0, 0.0]` | 从臂退出归位位置 [rad] |
| `grip_stable_pos` | `[0.5]` | 夹爪稳定位置（已知夹爪型号时取限幅中点） |
| `arm_stable_kp` | `[200.0, 200.0, 250.0, 150.0, 100.0, 100.0]` | 从臂稳定运动 PD 增益 — 比例 |
| `arm_stable_kd` | `[5.0, 5.0, 5.0, 5.0, 2.0, 2.0]` | 从臂稳定运动 PD 增益 — 微分 |
| `grip_stable_kp` | `[10.0]` | 夹爪稳定运动 PD 增益 — 比例 |
| `grip_stable_kd` | `[0.5]` | 夹爪稳定运动 PD 增益 — 微分 |
| `arm_slave_kp` | `[200.0, 200.0, 250.0, 200.0, 100.0, 100.0]` | 从臂跟随 PD 增益 — 比例 |
| `arm_slave_kd` | `[5.0, 5.0, 5.0, 5.0, 2.0, 2.0]` | 从臂跟随 PD 增益 — 微分 |
| `grip_slave_kp` | `[200.0]` | 从夹爪跟随 PD 增益 — 比例 |
| `grip_slave_kd` | `[1.0]` | 从夹爪跟随 PD 增益 — 微分 |
| `robot_grip_type` | `gr100` | 从臂夹爪型号：`gp100` / `gp80` / `gr100` / `empty` |
| `velocity_coupling_coeff` | `1.0` | 主臂速度前馈耦合系数 |
| `error_proportional_gain` | `1.0` | 动态比例增益误差系数（`tanh` 饱和） |
| `arm_kmin` / `arm_kmax` | `10.0` / `200.0` | 从臂动态比例增益范围 |
| `grip_kmin` / `grip_kmax` | `10.0` / `20.0` | 夹爪动态比例增益范围 |

> 表中默认值来自 `arm_follow.yaml` 的节点参数。完整 launch 会把命令行中的 `robot_grip_type` 覆盖到节点；本文真机命令示例使用 `empty`。

## 项目结构

```text
hex_ros_demo_arm_follow/
├── config/
│   ├── ros1/
│   │   └── arm_follow.yaml                  # ROS 1 节点参数
│   └── ros2/
│       └── arm_follow.yaml                  # ROS 2 节点参数
├── hex_ros_demo_arm_follow/
│   ├── utility/
│   │   ├── __init__.py                      # utility 包初始化文件
│   │   ├── interface_base.py                # ROS 接口基类
│   │   ├── ros1_interface.py                # ROS 1 接口
│   │   └── ros2_interface.py                # ROS 2 接口
│   ├── __init__.py                          # Python 包初始化文件
│   ├── arm_follow.py                        # 跟随节点
│   └── TrajectoryController.py              # 轨迹控制类
├── launch/
│   ├── ros1/
│   │   ├── arm_follow.launch                # ROS 1 跟随节点启动文件
│   │   ├── dual_hello2real_follow.launch    # ROS 1 双组真机启动文件
│   │   ├── hello2real_follow.launch         # ROS 1 单组真机启动文件
│   │   └── hello2sim_follow.launch          # ROS 1 真机与仿真启动文件
│   └── ros2/
│       ├── arm_follow.launch.py             # ROS 2 跟随节点启动文件
│       ├── dual_hello2real_follow.launch.py # ROS 2 双组真机启动文件
│       ├── hello2real_follow.launch.py      # ROS 2 单组真机启动文件
│       └── hello2sim_follow.launch.py       # ROS 2 真机与仿真启动文件
├── resource/
│   └── hex_ros_demo_arm_follow              
├── .gitignore                               
├── CMakeLists.txt                           
├── LICENSE                                  
├── package.xml                              
├── README_cn.md                             
├── README.md                                
├── setup.cfg                                
└── setup.py                                 
```
