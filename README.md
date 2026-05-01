# ROS2 Robotic Arm 7-DOF

Educational project: a complete ROS2 Humble control stack for a 7-DOF robotic arm (6 joints + gripper) driven by CubeMars motors over CAN.

## Tech stack

- **ROS2 Humble**
- **C++17** — hardware interface, CAN communication
- **Python** — launch files, direct serial trajectory tool
- **MoveIt2** — motion planning
- **ros2_control** — hardware abstraction layer

## Project structure

```
ros2_robotic_arm/
├── src/
│   ├── arm6dof/
│   │   ├── src/arm/
│   │   │   ├── ArmController.hpp/cpp         # Motor control: MIT protocol, per-motor params
│   │   │   ├── CanBridge.hpp/cpp             # Waveshare USB-CAN-A serial driver (std + ext frames)
│   │   │   └── ArmHardwareInterface.hpp/cpp  # ros2_control SystemInterface plugin
│   │   ├── arm6dof_plugin.xml
│   │   ├── launch/main.launch.py
│   │   └── urdf/arm6dof.urdf
│   └── arm6dof_moveit_config/
│       ├── config/                           # Kinematics, planners, controllers
│       └── launch/
└── tools/
    └── trajectory.py                         # Direct serial MIT tool (no ROS2)
```

## Architecture

```
MoveIt2 / RViz2
    └── JointTrajectoryController (ros2_control)
            └── ArmHardwareInterface  (ros2_control plugin)
                    └── ArmController
                            └── CanBridge → USB-CAN-A (2 Mbps) → motors
```

`write()` sends an MIT command to each motor and immediately waits for the response (synchronous). States are updated in `read()` on the next cycle.

## Motors

| ID | Model | CAN frame | kp_cmd | kd_cmd |
|----|-------|-----------|--------|--------|
| 1 | AK45-36 | standard 11-bit | 15.0 | 0.5 |
| 2 | AK60-39 | extended 29-bit | 0.5 | 0.1 |
| 3 | AK45-36 | standard 11-bit | 15.0 | 0.5 |
| 4 | AK45-36 | standard 11-bit | 15.0 | 0.5 |
| 5 | AK45-10 | standard 11-bit | 15.0 | 0.5 |
| 6 | AK45-10 | standard 11-bit | 15.0 | 0.5 |
| 7 | AK40-10 | standard 11-bit | 15.0 | 0.5 (gripper) |

Motor 2 (AK60-39) uses extended CAN frames: MIT command `CAN ID = 0x802`, feedback `CAN ID = 0x2902`, disable `CAN ID = 0xF02`.

Parameter ranges and default gains are defined per-model in `ArmController.hpp` (`MotorPresets` namespace).

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `joint_states` | `JointState` | pub | Position/velocity from motor feedback |
| `mit_enable` | `std_msgs/Int32` | sub | Enable motors: `0` = all, `1–7` = single |

## Build and run

```bash
cd ~/ros2_projekt/ros2_robotic_arm
colcon build
source install/setup.bash
ros2 launch arm6dof main.launch.py
```

Hardware interface opens `/dev/ttyUSB2` (configured in `ArmHardwareInterface.cpp`).

## Manual motor enable

```bash
# Enable all motors
ros2 topic pub --once /mit_enable std_msgs/msg/Int32 "{data: 0}"

# Enable motor 3 only
ros2 topic pub --once /mit_enable std_msgs/msg/Int32 "{data: 3}"
```

## Direct serial tool (no ROS2)

```bash
python3 tools/trajectory.py
```

Sends MIT frames directly over serial, bypassing ros2_control. Useful for testing individual motors. Edit `CAN_PORT`, `MOTOR_IDS`, `POS_A`/`POS_B` at the top of the file.

## Dependencies

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-moveit
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher
```

## Hardware

- **Waveshare USB-CAN-A** — CH340, binary serial framing, 2 Mbps
- **CAN protocol:** standard 11-bit MIT frames for most motors; extended 29-bit frames for AK60-39

## Project status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | ROS2 workspace, C++ node, launch file | ✅ |
| 2 | URDF 7-DOF, TF2, RViz2 visualization | ✅ |
| 3 | CanBridge (std + ext frames), ArmController MIT-only | ✅ |
| 4 | ros2_control Hardware Interface, MoveIt2 integration | ✅ |
| 5 | 7-motor support, AK60-39 extended frame protocol | ✅ |
| 6 | Motion execution with real hardware | ⏳ |
| 7 | Gazebo simulation | ⏳ |
| 8 | Tests, CI/CD, Docker | ⏳ |

## Robot

7-DOF manipulator (6 arm joints + gripper). CAD models in `urdf/meshes/`.
