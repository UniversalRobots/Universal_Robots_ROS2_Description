# Universal_Robots_ROS2_Description

This repository contains description files and meshes for *Universal Robots* manipulators.

### Build status

ROS2 Distro | Branch | Build status | Released packages
:---------: | :----: | :----------: | :---------------:
**Galactic** | [`galactic`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/galactic) | [![Galactic Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-galactic.yml/badge.svg?branch=galactic)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-galactic.yml?branch=galactic) <br /> | [ur_description](https://index.ros.org/p/ur_description/#galactic)
**Humble** | [`ros2`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2) | [![Humble Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-humble.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-humble.yml?branch=ros2) <br /> | [ur_description](https://index.ros.org/p/ur_description/#humble)
**Iron** | [`ros2`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2) | [![Iron Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-iron.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-iron.yml?branch=ros2) <br /> | [ur_description](https://index.ros.org/p/ur_description/#iron)
**Rolling** | [`ros2`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2) | [![Rolling Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-rolling.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/actions/workflows/binary-rolling.yml?branch=ros2) <br /> | [ur_description](https://index.ros.org/p/ur_description/#rolling)


Note that for ROS2 **Foxy** the description is in the [driver's
repository](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy). Please do
not clone this repository into a Foxy workspace.

## Structure of the repository

The most relevant files are:
  - `urdf/ur_macro.xacro` - macro file with UR-manipulator description. This file is usually included into external projects to visualize and configure UR manipulators properly. An example how to use this macro is in `urdf/ur.urdf.xacro` file.
  - `urdf/ur.ros2_control.xacro` - definition of manipulator's joints and interfaces for `ros2_control` framework.

## Testing description of a manipulator

To visualize the robot install this repository to you workspace and execute the following:
```
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
```

To test other descriptions change the `ur_type` argument.

## Package / Description structure
This package uses one description for all robots. The different robot variants are configured using
four configuration files. These files can also be changed for further customizing a description.

![urdf structure](doc/structure.svg)

Basically, the description can be modified using configuration values stored in four files:
 - `config/urXX/default_kinematics.yaml` - This contains the calibration values as they can be
   extracted from the robot. Changing these values with the one extracted from a real robot will
   result in a description matching the real robot exactly (w.r.t the `tool0` frame). It is highly
   recommended to use matching kinematic values in real-world applications.
 - `config/urXX/joint_limits.yaml` - If you'd like to further restrict the robot's joint limits,
   these limits can be modified there.
 - `config/urXX/physical_parameters.yaml` - Everything regarding physics simulation parameters
   (e.g. inertia poses and values) can be tuned here
 - `config/urXX/visual_parameters.yaml` - Some users change certain visual aspects, e.g. replacing
   the cap on the wrist_3_link. This config file specifies which meshes (both, visual and collision)
   should be used.

The four configuration files have to be passed to `ur_macro.urdf` (more specific to the macro
defined in that file) which is done inside the `ur.urdf.macro`. Contents of the files are parsed
inside `ur_common.xacro`.

Arguments that have to be passed to the main `ur.urdf.xacro` file are:
 - kinematics_params - Filename to the `default_kinematics.yaml` (or equivalent specific kinematics) file
 - joint_limit_params - Filename to the `joint_limits.yaml` file
 - physical_params - Filename to the `physical_parameters.yaml` file
 - visual_params - Filename to the `visual_params.yaml` file

The launchfile `launch/view_ur.launch.py` abstracts these four parameters to one `ur_type` argument
which will basically replace the `urXX` part of the paths as shown in the picture above.

## Creating your own description including this description
In real-world applications you will most probably have a more complex description consisting of more
objects than just the robot. It is recommended to create a separate ROS package containing this
particular description. Inside this description you could also store your robot-specific kinematics
parameters file.

As mentioned above, see the `urdf/ur.urdf.xacro` file as an example to integrate a UR robot into
your scene description. Basically, you could create a copy of that file and extend it with the
modifications from your specific scene.
