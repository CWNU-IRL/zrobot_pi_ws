# zrobot_gz_sim

Temporary Gazebo simulation package for zrobot.

## Features

- Temporary URDF with 23 revolute joints and one IMU.
- ros2_control integration for joint group position control.
- Service bridge compatible with existing interfaces:
  - /rob_stride_control (rs_interface/srv/RobStrideMsgs)
  - /get_positions (rs_interface/srv/GetPositions)
  - /set_zeros (rs_interface/srv/SetZeros)

## Build

```bash
colcon build --packages-select zrobot_gz_sim
```

## Run

```bash
source install/setup.bash
ros2 launch zrobot_gz_sim sim_bringup.launch.py
```

## Validate services

```bash
ros2 service list | grep -E 'rob_stride_control|get_positions|set_zeros'
```
