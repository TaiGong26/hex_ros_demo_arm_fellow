# hex_ros_demo_arm_follow
[中文](README_cn.md) | **English**

## Table of Contents

- [1. About](#1-about)
- [2. Package Structure](#2-package-structure)
- [3. Topics](#3-topics)
- [4. Parameters](#4-parameters)
- [5. Dependencies](#5-dependencies)
- [6. Quick Start](#6-quick-start)

---

## 1. About

This is the **master-slave follow demo** for the **HEXFELLOW** Archer Y6 robotic arm. The **master arm is always a real Hello Y6** (read-only, hand-held by the operator); the slave arm is a real or simulated Archer Y6.

The master-slave follow control loop centers on `__follow`, performing the following functions:

- **Master-slave follow control** — At each cycle, reads real-time joint states from both the master arm (Hello Y6) and slave arm (Archer Y6), computes follow commands with an error-adaptive dynamic proportional gain (`tanh` saturation), coupled with the master arm's velocity feedforward, and publishes MIT impedance control commands (`__build_follow_ctrl`, slave-side PD gains + gravity compensation), so the slave arm smoothly follows the master arm's position.
- **Master arm read-only** — The master arm is a read-only Hello Y6 (no `manip_ctrl` commands are published to it), manually moved by the operator.
- **Gripper follow** — The 1-DoF slave gripper is controlled by the master arm handle's trigger, clipped to the gripper limits by model (`robot_grip_type`); gripper follow is disabled when the model is unknown.
- **Smooth start/exit** — On start, the slave arm slowly approaches the master arm's position to go online (exponential approach, τ = 0.8 s); on exit, it smoothly returns from its current position to the stable position via the trajectory planner.
- **Keyboard control** — Press **`q`** to stop follow control; the slave arm returns to home position and exits.

Supports both **ROS 1** and **ROS 2**, with two full launch scenarios: real-to-simulation (hello2sim, master = real Hello, slave = simulation) and real-to-real (hello2real, master = real Hello, slave = real Archer).

---

## 2. Package Structure

```
hex_ros_demo_arm_follow/
├── config/                                  # Configuration files
│   ├── ros1/
│   │   └── arm_follow.yaml                  #   ROS 1 params (follow)
│   └── ros2/
│       └── arm_follow.yaml                  #   ROS 2 params (follow)
├── launch/                                  # ROS launch files
│   ├── ros1/
│   │   ├── arm_follow.launch                #   Standalone node launch
│   │   ├── hello2real_follow.launch         #   Real-to-real full launch
│   │   └── hello2sim_follow.launch          #   Real-to-sim full launch
│   └── ros2/
│       ├── arm_follow.launch.py             #   Standalone node launch
│       ├── hello2real_follow.launch.py      #   Real-to-real full launch
│       └── hello2sim_follow.launch.py       #   Real-to-sim full launch
├── hex_ros_demo_arm_follow/                 # Core source
│   ├── arm_follow.py                        #   Main node: master-slave follow control loop
│   ├── TrajectoryController.py              #   Trajectory planner
│   └── utility/                             #   Dual-layer ROS interface abstraction
│       ├── __init__.py                      #     ROS version selector (ROS_VERSION env var)
│       ├── interface_base.py                #     Abstract base class (InterfaceBase)
│       ├── ros1_interface.py                #     ROS 1 DataInterface
│       └── ros2_interface.py                #     ROS 2 DataInterface
├── resource/                                # ament resource index
├── setup.py                                 # Python packaging (ROS 2)
├── CMakeLists.txt                           # CMake packaging (ROS 1)
├── package.xml                              # ROS package manifest (dual-system conditional deps)
├── README.md                                # English documentation
└── README_cn.md                             # Chinese documentation
```

---

## 3. Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| pub | `slave/manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | Slave arm MIT follow control command (arm + gripper) |
| pub | `master/color_cmd` | `std_msgs/(msg/)ColorRGBA` | Master arm LED status indication (init/follow phases) |
| sub | `master/manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | Master arm (Hello Y6) real-time state |
| sub | `slave/manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | Slave arm (Archer Y6) real-time state |
| sub | `master/joy_state` | `hex_ros_msgs/(msg/)HexRosTeleopHandleStateStamped` | Handle state (trigger → gripper follow) |
| sub | `teleop_keyboard_state` | `hex_ros_msgs/(msg/)HexRosTeleopKeyboardStateStamped` | Keyboard key state |

> `master/manip_ctrl` is not published — the master Hello Y6 is a read-only device.
> [Message Type Description](https://github.com/hexfellow/hex_ros_msgs#public-apis)

---

## 4. Parameters

| Param | Default | Description |
|-------|---------|-------------|
| `rate_ros` | 1000.0 | Follow control loop rate [Hz] |
| `rate_teleop` | 100.0 | Keyboard monitor rate [Hz] |
| `model_urdf` | "" | URDF model file path (set by launch, not used by the node) |
| `model_frame_id` | `base_link` | Robot base frame ID |
| `gravity` | `[0.0, 0.0, -9.81]` | Gravity acceleration vector [m/s²] |
| `arm_end_pos` | `[0.0, -1.5, 3.0, 0.07, 0.0, 0.0]` | Slave arm exit home position [rad] |
| `grip_stable_pos` | `[0.5]` | Gripper stable position (midpoint of the limits when the gripper model is known) |
| `arm_stable_kp` | `[200.0, 200.0, 250.0, 150.0, 100.0, 100.0]` | Slave arm stable motion PD gain — proportional |
| `arm_stable_kd` | `[5.0, 5.0, 5.0, 5.0, 2.0, 2.0]` | Slave arm stable motion PD gain — derivative |
| `grip_stable_kp` | `[10.0]` | Gripper stable motion PD gain — proportional |
| `grip_stable_kd` | `[0.5]` | Gripper stable motion PD gain — derivative |
| `arm_slave_kp` | `[200.0, 200.0, 250.0, 200.0, 100.0, 100.0]` | Slave arm follow PD gain — proportional |
| `arm_slave_kd` | `[5.0, 5.0, 5.0, 5.0, 2.0, 2.0]` | Slave arm follow PD gain — derivative |
| `grip_slave_kp` | `[200.0]` | Slave gripper follow PD gain — proportional |
| `grip_slave_kd` | `[1.0]` | Slave gripper follow PD gain — derivative |
| `robot_grip_type` | `gr100` | Slave gripper model (gp100 / gp80 / gr100 / empty) |
| `velocity_coupling_coeff` | 1.0 | Master arm velocity feedforward coupling coefficient |
| `error_proportional_gain` | 1.0 | Dynamic gain error coefficient (`tanh` saturation) |
| `arm_kmin` / `arm_kmax` | 10.0 / 200.0 | Slave arm dynamic proportional gain range |
| `grip_kmin` / `grip_kmax` | 10.0 / 20.0 | Gripper dynamic proportional gain range |

> Parameters are set in `config/`. Defaults are identical between ROS 1 and ROS 2; `model_urdf` / `model_frame_id` are set by the launch file but not used by the follow node.

---

## 5. Dependencies

### Python Packages

```shell
pip3 install 'hex-util-msg>=0.1.0'
pip3 install 'hex-util-ros>=0.1.0a4'
pip3 install 'hex-driver-robot>=0.1.0'
```

### ROS Packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_follow.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

---

## 6. Quick Start

### 1. Create Workspace

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 2. Clone Repositories

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_follow.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

### 3. Build

**ROS 1:**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

**ROS 2:**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

### 4. Use

This package provides two full launch scenarios plus a standalone node launch. PD gains and dynamic gain ranges and other parameters are configured in `config/<ros_version>/arm_follow.yaml`.

**ROS 2:**

```shell
# Real-to-simulation (hello2sim): master is real Hello Y6 (read-only), slave is Archer simulation
ros2 launch hex_ros_demo_arm_follow hello2sim_follow.launch.py \
    master_robot_host:=<hello_ip> master_robot_port:=8439 viewer:=true rviz:=false

# Real-to-real (hello2real): master is real Hello Y6 (read-only), slave is real Archer Y6
ros2 launch hex_ros_demo_arm_follow hello2real_follow.launch.py \
    master_robot_host:=<hello_ip> master_robot_port:=8439 \
    slave_robot_host:=<archer_ip> slave_robot_port:=8439 robot_grip_type:=gr100 robot_type:=archer

# Start follow node only (requires separate arm state/control drivers)
ros2 launch hex_ros_demo_arm_follow arm_follow.launch.py
```

**ROS 1:**

```shell
# Real-to-simulation (hello2sim)
roslaunch hex_ros_demo_arm_follow hello2sim_follow.launch master_robot_host:=<hello_ip> viewer:=true rviz:=false

# Real-to-real (hello2real)
roslaunch hex_ros_demo_arm_follow hello2real_follow.launch master_robot_host:=<hello_ip> slave_robot_host:=<archer_ip> robot_grip_type:=gr100 robot_type:=archer

# Start follow node only
roslaunch hex_ros_demo_arm_follow arm_follow.launch robot_grip_type:=gr100
```

> Ensure parameters in `config/` are correctly set. The URDF path is set automatically by the launch file.

Keyboard control:

- **`q`** — Stop follow control; slave arm returns to home position and exits (follow starts automatically after the slave arm goes online — no key needed)
