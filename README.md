# Universal_Robots_ROS2_Description

This repository containes description files and meshes for *Universal Robots* manipulators.

### Build status

ROS2 Distro | Branch | Build status | Released packages
:---------: | :----: | :----------: | :---------------:
**Foxy** | [`ros2`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2) | [![Foxy Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-build.yml?branch=ros2) <br /> | [ur_description](https://index.ros.org/p/ur_description/#foxy)
**Galactic** | [`ros2`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2) | [![Galactic Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-build.yml?branch=ros2) <br /> | [ur_description](https://index.ros.org/p/ur_description/#galactic)
**Rolling** | [`ros2`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2) | [![Rolling Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-build.yml?branch=ros2) <br /> | [ur_description](https://index.ros.org/p/ur_description/#rolling)


## Structure of the repository

The most relevant files are:
  - `urdf/ur_macro.xacro` - macro file with UR-manipulator description. This file is usually included into external projects to visualize and configure UR manipulators properly. An example how to use this macro is in `urdf/ur.urdf.xacro` file.
  - `urdf/ur.ros2_control.xacro` - definiton of manipulator's joints and interfaces for `ros2_control` framework.

## Testing description of a manipulator

To visualize the robot install this repository to you workspace and execute the following:
```
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
```

To test other descriptions change the `ur_type` argument.
