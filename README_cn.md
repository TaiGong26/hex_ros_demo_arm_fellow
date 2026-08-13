# hex_ros_demo_arm_follow
**中文** | [English](README.md)

## 目录

- [1. 包的简介](#1-包的简介)
- [2. 包架构](#2-包架构)
- [3. 话题接口](#3-话题接口)
- [4. 参数说明](#4-参数说明)
- [5. 依赖关系](#5-依赖关系)
- [6. 快速使用](#6-快速使用)

---

## 1. 包的简介

这是 **HEXFELLOW** Archer Y6 机械臂的**主从跟随演示包**（master-slave follow demo）。**主臂永远是真机 Hello Y6**（只读，由操作者手持）；从臂是 Archer Y6 真机或仿真。

本包在主从跟随控制循环中以 `__follow` 为核心，完成以下功能：

- **主从跟随控制** — 每周期读取主臂（Hello Y6）和从臂（Archer Y6）的实时关节状态，以按误差自适应调节的动态比例增益（`tanh` 饱和）计算跟随指令，并耦合主臂速度前馈，发布 MIT 阻抗控制指令（`__build_follow_ctrl`，从端 PD 增益 + 重力补偿），使从臂平滑跟随主臂位置。
- **主臂只读** — 主臂是只读 Hello Y6（不向其发布任何 `manip_ctrl` 指令），由操作者手动拖动。
- **夹爪随动** — 1-DOF 从臂夹爪由主臂手柄的扳机控制，按夹爪型号（`robot_grip_type`）查表限幅；未知型号时禁用夹爪随动。
- **缓启动/缓退出** — 启动时从臂从当前位置缓慢逼近主臂位置"上线"（指数逼近，τ = 0.8 s）；退出时经轨迹规划器从当前位置平滑归位到稳定位置。
- **键盘控制** — 按 **`q`** 停止跟随控制，从臂归位后退出。

同时支持 **ROS 1** 和 **ROS 2**，提供两种完整启动场景：真机对仿真（hello2sim，主臂=真机 Hello，从臂=仿真）和真机对真机（hello2real，主臂=真机 Hello，从臂=真机 Archer）。

---

## 2. 包架构

```
hex_ros_demo_arm_follow/
├── config/                                  # 参数配置
│   ├── ros1/
│   │   └── arm_follow.yaml                  #   ROS 1 参数（follow）
│   └── ros2/
│       └── arm_follow.yaml                  #   ROS 2 参数（follow）
├── launch/                                  # ROS launch 启动文件
│   ├── ros1/
│   │   ├── arm_follow.launch                #   单节点启动
│   │   ├── hello2real_follow.launch         #   真机对真机完整启动
│   │   └── hello2sim_follow.launch          #   真机对仿真完整启动
│   └── ros2/
│       ├── arm_follow.launch.py             #   单节点启动
│       ├── hello2real_follow.launch.py      #   真机对真机完整启动
│       └── hello2sim_follow.launch.py       #   真机对仿真完整启动
├── hex_ros_demo_arm_follow/                 # 核心代码
│   ├── arm_follow.py                        #   主节点：主从跟随控制循环
│   ├── TrajectoryController.py              #   轨迹规划器
│   └── utility/                             #   双层 ROS 接口抽象层
│       ├── __init__.py                      #     ROS 版本选择器（ROS_VERSION 环境变量）
│       ├── interface_base.py                #     抽象基类（InterfaceBase）
│       ├── ros1_interface.py                #     ROS 1 DataInterface
│       └── ros2_interface.py                #     ROS 2 DataInterface
├── resource/                                # ament 资源索引
├── setup.py                                 # Python 打包配置（ROS 2）
├── CMakeLists.txt                           # CMake 打包配置（ROS 1）
├── package.xml                              # ROS 包清单（双系统条件依赖）
├── README.md                                # 英文文档
└── README_cn.md                             # 中文文档
```

---

## 3. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `slave/manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | 从臂 MIT 跟随控制指令（臂 + 夹爪） |
| 发布 | `master/color_cmd` | `std_msgs/(msg/)ColorRGBA` | 主臂 LED 状态指示（初始化/跟随阶段） |
| 订阅 | `master/manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | 主臂（Hello Y6）实时状态 |
| 订阅 | `slave/manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | 从臂（Archer Y6）实时状态 |
| 订阅 | `master/joy_state` | `hex_ros_msgs/(msg/)HexRosTeleopHandleStateStamped` | 手柄状态（扳机 → 夹爪随动） |
| 订阅 | `teleop_keyboard_state` | `hex_ros_msgs/(msg/)HexRosTeleopKeyboardStateStamped` | 键盘按键状态 |

> 不发布 `master/manip_ctrl` —— 主臂 Hello Y6 为只读设备。
> [消息类型描述](https://github.com/hexfellow/hex_ros_msgs#public-apis)

---

## 4. 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rate_ros` | 1000.0 | 跟随控制循环频率 [Hz] |
| `rate_teleop` | 100.0 | 键盘监听频率 [Hz] |
| `model_urdf` | "" | URDF 模型文件路径（launch 自动设置，节点未使用） |
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
| `robot_grip_type` | `gr100` | 从臂夹爪型号（gp100 / gp80 / gr100 / empty） |
| `velocity_coupling_coeff` | 1.0 | 主臂速度前馈耦合系数 |
| `error_proportional_gain` | 1.0 | 动态比例增益误差系数（`tanh` 饱和） |
| `arm_kmin` / `arm_kmax` | 10.0 / 200.0 | 从臂动态比例增益范围 |
| `grip_kmin` / `grip_kmax` | 10.0 / 20.0 | 夹爪动态比例增益范围 |

> 参数在 `config/` 中设置，ROS 1 与 ROS 2 的默认值一致；`model_urdf` / `model_frame_id` 由 launch 文件设置，但跟随节点未使用。

---

## 5. 依赖关系

### Python 包

```shell
pip3 install 'hex-util-msg>=0.1.0'
pip3 install 'hex-util-ros>=0.1.0a4'
pip3 install 'hex-driver-robot>=0.2.0'
```

### ROS 包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_follow.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

---

## 6. 快速使用

### 1. 构建工作空间

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 2. 克隆包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_follow.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

### 3. 编译包

**ROS 1：**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

**ROS 2：**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

### 4. 使用包

本包提供两种完整启动场景的 launch 文件加单节点 launch，PD 增益和动态增益范围等参数在 `config/<ros_version>/arm_follow.yaml` 中配置。

**ROS 2：**

```shell
# 真机对仿真（hello2sim）：主臂是真机 Hello Y6（只读），从臂是 Archer 仿真
ros2 launch hex_ros_demo_arm_follow hello2sim_follow.launch.py \
    master_robot_host:=<hello_ip> master_robot_port:=8439 viewer:=true rviz:=false

# 真机对真机（hello2real）：主臂是真机 Hello Y6（只读），从臂是真机 Archer Y6
ros2 launch hex_ros_demo_arm_follow hello2real_follow.launch.py \
    master_robot_host:=<hello_ip> master_robot_port:=8439 \
    slave_robot_host:=<archer_ip> slave_robot_port:=8439 robot_grip_type:=gr100 robot_type:=archer

# 仅启动跟随节点（需自行提供主/从臂状态和控制驱动）
ros2 launch hex_ros_demo_arm_follow arm_follow.launch.py
```

**ROS 1：**

```shell
# 真机对仿真（hello2sim）
roslaunch hex_ros_demo_arm_follow hello2sim_follow.launch master_robot_host:=<hello_ip> viewer:=true rviz:=false

# 真机对真机（hello2real）
roslaunch hex_ros_demo_arm_follow hello2real_follow.launch master_robot_host:=<hello_ip> slave_robot_host:=<archer_ip> robot_grip_type:=gr100 robot_type:=archer

# 仅启动跟随节点
roslaunch hex_ros_demo_arm_follow arm_follow.launch robot_grip_type:=gr100
```

> 确保 `config/` 中的参数配置正确，URDF 路径由 launch 文件自动设置。

键盘控制：

- **`q`** — 停止跟随控制，从臂归位后退出（从臂上线完成后自动开始跟随，无需按键）
