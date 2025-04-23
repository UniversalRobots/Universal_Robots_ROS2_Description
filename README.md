# Universal_Robots_ROS2_Description

This repository contains description files and meshes for *Universal Robots* manipulators.

## Build status

ROS2 Distro | Branch | Build status | Released packages
:---------: | :----: | :----------: | :---------------:
**Humble** | [`humble`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble) | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__ur_description__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__ur_description__ubuntu_jammy_amd64__binary/) | [ur_description](https://index.ros.org/p/ur_description/#humble)
**Jazzy** | [`jazzy`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/jazzy) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__ur_description__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__ur_description__ubuntu_noble_amd64__binary/) | [ur_description](https://index.ros.org/p/ur_description/#jazzy)
**Kilted** | [`rolling`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/rolling) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__ur_description__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__ur_description__ubuntu_noble_amd64__binary/) | [ur_description](https://index.ros.org/p/ur_description/#kilted)
**Rolling** | [`rolling`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/rolling) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__ur_description__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__ur_description__ubuntu_noble_amd64__binary/)| [ur_description](https://index.ros.org/p/ur_description/#rolling)

A more [detailed build status](ci_status.md) shows the state of all CI workflows inside this repo.
Please note that the detailed view is intended for developers, while the one here should give end
users an overview of the current released state.


## License
The [UR15 meshes](meshes/ur15), [UR20 meshes](meshes/ur20) and [UR30 meshes](meshes/ur30) constitutes “Graphical Documentation” the use of which is subject to and governed by our “[Terms and Conditions for use of Graphical Documentation](https://www.universal-robots.com/legal/terms-and-conditions/terms_and_conditions_for_use_of_graphical_documentation.txt)”.

Universal Robots' [Terms and Conditions for use of Graphical Documentation](https://www.universal-robots.com/legal/terms-and-conditions/terms_and_conditions_for_use_of_graphical_documentation.txt) do not fully comply with [OSI's definition of Open Source](https://opensource.org/osd/), but they do allow you to use, modify and share “Graphical Documentation”, including [UR15 meshes](meshes/ur15), [UR20](meshes/ur20) and [UR30](meshes/ur30) meshes, subject to certain restrictions.\
If you have any questions regarding this license or if this license doesn't fit your use-case, please contact [legal@universal-robots.com](mailto:legal@universal-robots.com).

All other content is licensed under the BSD-3-Clause license

## Testing description of a manipulator

To visualize the robot install this repository to you workspace and execute the following:

``` bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
```

To test other descriptions change the `ur_type` argument.

## Further documentation

For further documentation about this description, please see [doc/index.rst](doc/index.rst).
