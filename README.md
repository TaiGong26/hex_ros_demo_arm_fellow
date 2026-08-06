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

This is the **master-slave follow demo** for the **HEXFELLOW** Archer Y6 robotic arm. The **master is always the real Hello Y6** (read-only, hand-held by the operator); the slave is either the Archer Y6 simulation or a real Archer Y6.

The follow control loop centers on `__follow`, performing the following functions:

- **Master-slave following** — At each cycle, reads the master (Hello Y6) and slave (Archer Y6) joint states, computes a per-joint clipped target for the slave relative to the master, and publishes a MIT impedance command so the slave follows the master position (`__build_follow_ctrl`, slave-side PD gains).
- **Read-only master** — The master is a read-only Hello Y6 (no `manip_ctrl` command is published to it); the operator physically moves it.
- **Smooth start/exit** — On start and exit, the slave smoothly moves from its current position to the stable position via a trajectory planner (`Move2TargetPlanner`).
- **Keyboard control** — Press **`s`** to start follow control, press **`q`** to stop and return the slave to home position.

Supports both **ROS 1** and **ROS 2**, with two full launch scenarios: real-to-simulation (real2sim, master = real Hello, slave = simulation) and real-to-real (real2real, master = real Hello, slave = real Archer).

> Grip following is `### TODO` — the slave gripper currently stays at `grip_stable_pos` (the master Hello Y6 has no gripper).

---

## 2. Package Structure

```
hex_ros_demo_arm_follow/
├── config/                                # Configuration files
│   ├── ros1/
│   │   └── arm_follow.yaml                #   ROS 1 params (follow)
│   └── ros2/
│       └── arm_follow.yaml                #   ROS 2 params (follow)
├── launch/                                # ROS launch files
│   ├── ros1/
│   │   ├── arm_follow.launch              #   Standalone node launch
│   │   ├── real2sim_follow.launch         #   Real-to-sim full launch
│   │   └── real2real_follow.launch        #   Real-to-real full launch
│   └── ros2/
│       ├── arm_follow.launch.py           #   Standalone node launch
│       ├── real2sim_follow.launch.py      #   Real-to-sim full launch
│       └── real2real_follow.launch.py     #   Real-to-real full launch
├── hex_ros_demo_arm_follow/               # Core source
│   ├── arm_follow.py                      #   Main node: master-slave follow control loop
│   ├── TrajectoryController.py            #   Trajectory planner
│   └── utility/                           #   Dual-layer ROS interface abstraction
│       ├── __init__.py                    #     ROS version selector (ROS_VERSION env var)
│       ├── interface_base.py              #     Abstract base class (InterfaceBase)
│       ├── ros1_interface.py              #     ROS 1 DataInterface
│       └── ros2_interface.py              #     ROS 2 DataInterface
├── resource/                              # ament resource index
├── setup.py                               # Python packaging (ROS 2)
├── CMakeLists.txt                         # CMake packaging (ROS 1)
├── package.xml                            # ROS package manifest (dual-system conditional deps)
├── README.md                              # English documentation
└── README_cn.md                           # Chinese documentation
```

---

## 3. Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| pub | `slave/manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | Slave arm MIT follow control command (arm + gripper) |
| sub | `master/manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | Master (Hello Y6) arm real-time state |
| sub | `slave/manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | Slave (Archer Y6) arm real-time state |
| sub | `teleop_keyboard_state` | `hex_ros_msgs/(msg/)HexRosTeleopKeyboardStateStamped` | Keyboard key state |

> No `master/manip_ctrl` is published — the master Hello Y6 is read-only.
> [Message Type Description](https://github.com/hexfellow/hex_ros_msgs#public-apis)

---

## 4. Parameters

| Param | Default | Description |
|-------|---------|-------------|
| `rate_ros` | 1000.0 | Follow control loop rate [Hz] |
| `rate_teleop` | 100.0 | Keyboard monitor rate [Hz] |
| `gravity` | `[0.0, 0.0, -9.81]` | Gravity acceleration vector [m/s²] |
| `arm_start_pos` | `[0.0, 0.1, 2.54, -1.07, 0.0, 0.0]` | Slave stable start position [rad] |
| `arm_end_pos` | `[0.0, -1.5, 3.0, 0.07, 0.0, 0.0]` | Slave exit home position [rad] |
| `grip_stable_pos` | `[0.5]` | Gripper stable position |
| `arm_stable_kp` | `[200.0, 200.0, 250.0, 150.0, 100.0, 100.0]` | Slave stable motion PD gain — proportional |
| `arm_stable_kd` | `[5.0, 5.0, 5.0, 5.0, 2.0, 2.0]` | Slave stable motion PD gain — derivative |
| `grip_stable_kp` | `[10.0]` | Gripper stable motion PD gain — proportional |
| `grip_stable_kd` | `[0.5]` | Gripper stable motion PD gain — derivative |
| `arm_slave_kp` | `[200.0, 200.0, 250.0, 200.0, 100.0, 100.0]` | Slave follow PD gain — proportional |
| `arm_slave_kd` | `[5.0, 5.0, 5.0, 5.0, 2.0, 2.0]` | Slave follow PD gain — derivative |
| `grip_slave_kp` | `[200.0]` | Slave gripper follow PD gain — proportional |
| `grip_slave_kd` | `[10.0]` | Slave gripper follow PD gain — derivative |
| `arm_slave_clip` | `[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]` | Slave arm error saturation clip [rad] |
| `grip_slave_clip` | `[0.3]` | Slave gripper error saturation clip [rad] |

> `model_urdf` / `model_frame_id` / `pose_end_in_flange` are still loaded (set by the launch file) but not used by the follow node. Master-side feedback params (`arm_master_*`, `*_deadzone`, `extra_mass`) remain in `config/` for reference but are unused.

---

## 5. Dependencies

### Python Packages

```shell
pip3 install 'hex-util-msg>=0.1.0'
pip3 install 'hex-util-ros>=0.0.1a0'
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

This package provides launch files for two full scenarios plus a standalone node. PD gains, clip, and other parameters are configured in `config/<ros_version>/arm_follow.yaml`.

**ROS 2:**

```shell
# Real-to-simulation (real2sim): master is the real Hello Y6 (read-only), slave is the Archer simulation
ros2 launch hex_ros_demo_arm_follow real2sim_follow.launch.py \
    master_robot_host:=<hello_ip> master_robot_port:=8439 viewer:=true rviz:=false

# Real-to-real (real2real): master is the real Hello Y6 (read-only), slave is a real Archer Y6
ros2 launch hex_ros_demo_arm_follow real2real_follow.launch.py \
    master_robot_host:=<hello_ip> master_robot_port:=8439 \
    slave_robot_host:=<archer_ip> slave_robot_port:=9439 robot_grip_type:=gr100

# Start the follow node only (requires separate arm state/control drivers)
ros2 launch hex_ros_demo_arm_follow arm_follow.launch.py
```

**ROS 1:**

```shell
# Real-to-simulation (real2sim)
roslaunch hex_ros_demo_arm_follow real2sim_follow.launch master_robot_host:=<hello_ip> viewer:=true rviz:=false

# Real-to-real (real2real)
roslaunch hex_ros_demo_arm_follow real2real_follow.launch master_robot_host:=<hello_ip> slave_robot_host:=<archer_ip>

# Start the follow node only
roslaunch hex_ros_demo_arm_follow arm_follow.launch
```

> Ensure parameters in `config/` are correctly set. The URDF path is set automatically by the launch file.

Keyboard control:

- **`s`** — Start follow control
- **`q`** — Stop follow control; the slave returns to home position and exits
