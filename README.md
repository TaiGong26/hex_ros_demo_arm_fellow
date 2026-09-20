# hex_ros_demo_arm_follow
[中文](README_cn.md) | **English**

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Installation](#installation)
- [Topics](#topics)
- [Parameters](#parameters)
- [Project Structure](#project-structure)

## Overview

`hex_ros_demo_arm_follow` demonstrates master-slave following from a Hello Y6 to an Archer Y6 or Firefly Y6. The operator moves the Hello Y6 to provide master state, while the slave receives commands that follow the arm joints and gripper input.

It provides:

- real Hello Y6 to simulated Archer Y6 following;
- single or dual real master-slave setups;
- a standalone follow node for custom integration.

This package supports **ROS 2 Humble** and is compatible with **ROS 1 Noetic**.

## Quick Start

> Complete [Installation](#installation) before launching.

### 1. Launch scenarios

#### Real-to-simulation (hello2sim)

This scenario connects a real Hello Y6 master to a simulated Archer Y6 slave for follow-logic validation.

ROS 2:

```shell
ros2 launch hex_ros_demo_arm_follow hello2sim_follow.launch.py \
    master_robot_host:=192.168.1.100 master_robot_port:=8439
```

ROS 1:

```shell
roslaunch hex_ros_demo_arm_follow hello2sim_follow.launch \
    master_robot_host:=192.168.1.100 master_robot_port:=8439
```

#### Single real-to-real (hello2real)

Master arm is a real Hello Y6; the slave may be a real Archer Y6 or Firefly Y6.

ROS 2:

```shell
ros2 launch hex_ros_demo_arm_follow hello2real_follow.launch.py \
    master_robot_host:=192.168.1.100 master_robot_port:=8439 \
    slave_robot_host:=192.168.1.101 slave_robot_port:=8439 \
    robot_type:=archer robot_grip_type:=empty enable_keyboard:=true
```

ROS 1:

```shell
roslaunch hex_ros_demo_arm_follow hello2real_follow.launch \
    master_robot_host:=192.168.1.100 master_robot_port:=8439 \
    slave_robot_host:=192.168.1.101 slave_robot_port:=8439 \
    robot_type:=archer robot_grip_type:=empty enable_keyboard:=true
```

#### Dual real-to-real (dual_hello2real)

Two master-slave groups run simultaneously, one on the left and one on the right.

ROS 2:

```shell
ros2 launch hex_ros_demo_arm_follow dual_hello2real_follow.launch.py \
    left_master_robot_host:=192.168.1.100 left_master_robot_port:=8439 \
    left_slave_robot_host:=192.168.1.101 left_slave_robot_port:=8439 \
    left_robot_type:=archer left_robot_grip_type:=empty \
    right_master_robot_host:=192.168.1.100 right_master_robot_port:=9439 \
    right_slave_robot_host:=192.168.1.101 right_slave_robot_port:=9439 \
    right_robot_type:=archer right_robot_grip_type:=empty enable_keyboard:=true
```

ROS 1:

```shell
roslaunch hex_ros_demo_arm_follow dual_hello2real_follow.launch \
    left_master_robot_host:=192.168.1.100 left_master_robot_port:=8439 \
    left_slave_robot_host:=192.168.1.101 left_slave_robot_port:=8439 \
    left_robot_type:=archer left_robot_grip_type:=empty \
    right_master_robot_host:=192.168.1.100 right_master_robot_port:=9439 \
    right_slave_robot_host:=192.168.1.101 right_slave_robot_port:=9439 \
    right_robot_type:=archer right_robot_grip_type:=empty enable_keyboard:=true
```

### 2. Keyboard control

- Follow starts automatically after the slave arm goes online — no key needed.
- Press **`q`** to stop follow control; the slave arm returns to home position and exits.

For dual launches, `enable_keyboard` controls one shared keyboard node in the root namespace:

- Set it to `false` to disable the keyboard node.
- Both follow nodes use `/teleop_keyboard_state`.

> Dual launches use `/left/master/*`, `/left/slave/*`, `/right/master/*`, and `/right/slave/*`. Both follow nodes use the shared `/teleop_keyboard_state` topic.

## Installation

### Prerequisites

- **ROS 2 Humble** is installed; use **ROS 1 Noetic** for ROS 1 compatibility.
- Python 3, `pip3`, Git, and the build tools for the selected ROS version are installed.
- For real-hardware scenarios, devices are reachable and the actual IP addresses, ports, and device models are known.

### 1. Install Python Dependencies

```shell
pip3 install \
    'hex-util-msg>=0.1.0' \
    'hex-util-ros>=0.1.0' \
    'hex-driver-robot>=0.1.0'
```

### 2. Create and Enter the Workspace

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 3. Clone ROS Packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_follow.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

### 4. Build

**ROS 2:**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

**ROS 1:**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

## Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| pub | `slave/manip_ctrl` | `hex_ros_msgs/msg/HexRosRoboManipCtrlStamped` | Slave arm control message |
| pub | `master/color_cmd` | `std_msgs/msg/ColorRGBA` | Master arm color message |
| sub | `master/manip_state` | `hex_ros_msgs/msg/HexRosRoboManipStateStamped` | Master arm state message |
| sub | `slave/manip_state` | `hex_ros_msgs/msg/HexRosRoboManipStateStamped` | Slave arm state message |
| sub | `master/joy_state` | `hex_ros_msgs/msg/HexRosTeleopHandleStateStamped` | Master arm handle state message |
| sub | `teleop_keyboard_state` | `hex_ros_msgs/msg/HexRosTeleopKeyboardStateStamped` | Keyboard state message |

> The Hello Y6 master provides state and handle input; control commands are published to the slave.
>
> Message type description: [hex_ros_msgs public APIs](https://github.com/hexfellow/hex_ros_msgs#public-apis)

## Parameters

### Launch Arguments

| Argument | Meaning / Values |
|---|---|
| `master_robot_host / master_robot_port` | Master IP / port |
| `slave_robot_host / slave_robot_port` | Slave IP / port; complete real launches only |
| `robot_type` | `archer` or `firefly` |
| `robot_grip_type` | gp100 / gp80 / gr100 / empty |
| `viewer / rviz` | Simulation window switches; simulation scenarios only |
| `enable_keyboard` | Keyboard switch for complete real launches; dual-arm connection arguments use left_ / right_ prefixes |

### Node Parameters

Parameters are set in `config/ros1/arm_follow.yaml` and `config/ros2/arm_follow.yaml`. Defaults are identical between ROS 1 and ROS 2.

| Param | Default | Description |
|-------|---------|-------------|
| `rate_ros` | `1000.0` | Follow control loop rate [Hz] |
| `rate_teleop` | `100.0` | Keyboard monitor rate [Hz] |
| `model_urdf` | `""` | URDF model file path (set by launch, not used by the node) |
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
| `robot_grip_type` | `gr100` | Slave gripper model (`gp100` / `gp80` / `gr100` / `empty`) |
| `velocity_coupling_coeff` | `1.0` | Master arm velocity feedforward coupling coefficient |
| `error_proportional_gain` | `1.0` | Dynamic gain error coefficient (`tanh` saturation) |
| `arm_kmin` / `arm_kmax` | `10.0` / `200.0` | Slave arm dynamic proportional gain range |
| `grip_kmin` / `grip_kmax` | `10.0` / `20.0` | Gripper dynamic proportional gain range |

> Defaults in this table come from the node parameters in `arm_follow.yaml`. Complete launches override the node's `robot_grip_type` with the command-line launch argument; the real-robot examples in this document use `empty`.

## Project Structure

```text
hex_ros_demo_arm_follow/
├── config/
│   ├── ros1/
│   │   └── arm_follow.yaml                  # ROS 1 node parameters
│   └── ros2/
│       └── arm_follow.yaml                  # ROS 2 node parameters
├── hex_ros_demo_arm_follow/
│   ├── utility/
│   │   ├── __init__.py                      # utility package initializer
│   │   ├── interface_base.py                # ROS interface base class
│   │   ├── ros1_interface.py                # ROS 1 interface
│   │   └── ros2_interface.py                # ROS 2 interface
│   ├── __init__.py                          # Python package initializer
│   ├── arm_follow.py                        # Follow node
│   └── TrajectoryController.py              # Trajectory control classes
├── launch/
│   ├── ros1/
│   │   ├── arm_follow.launch                # ROS 1 follow-node launch file
│   │   ├── dual_hello2real_follow.launch    # ROS 1 dual real-arm launch file
│   │   ├── hello2real_follow.launch         # ROS 1 single real-arm launch file
│   │   └── hello2sim_follow.launch          # ROS 1 real-to-simulation launch file
│   └── ros2/
│       ├── arm_follow.launch.py             # ROS 2 follow-node launch file
│       ├── dual_hello2real_follow.launch.py # ROS 2 dual real-arm launch file
│       ├── hello2real_follow.launch.py      # ROS 2 single real-arm launch file
│       └── hello2sim_follow.launch.py       # ROS 2 real-to-simulation launch file
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
