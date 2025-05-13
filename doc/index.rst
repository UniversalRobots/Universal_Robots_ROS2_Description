:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/jazzy/doc/index.rst

.. _ur_description:

ur_description
==============

This package contains the kinematic and visual description of Universal Robots manipulators for ROS
2.

.. toctree::
   :maxdepth: 4
   :caption: Contents:

   robot_frames

.. contents::
   :depth: 2
   :local:

Structure of the repository
---------------------------

As the kinematics of all models is roughly the same and varies only in the meshes, the DH
parameters and the physical parameters, the description uses xacro macros in order to make it reuse
as much as possible. This way it is less likely that errors are introduced in one of the models.

Therefore, the description is split up into multiple URDF/XACRO files:

- ``urdf/ur_macro.xacro`` - macro file with UR-manipulator description. This file is usually included into external projects to visualize and configure UR manipulators properly. An example how to use this macro is in ``urdf/ur.urdf.xacro`` file.
- ``urdf/ur.urdf.xacro`` - an example scene where the robot is standing alone without any
  environment. This is obviously not a real-life example, as the robot has to be mounted somewhere.

The kinematic description itself uses config files to get parametrized to the correct robot model.

.. image:: structure.svg
   :alt: Structure of the description's parametrization

Basically, the description can be modified using configuration values stored in four files:
 - ``config/urXX/default_kinematics.yaml`` - This contains the calibration values as they can be
   extracted from the robot. Changing these values with the one extracted from a real robot will
   result in a description matching the real robot exactly (w.r.t the ``tool0`` frame). It is highly
   recommended to use matching kinematic values in real-world applications.
 - ``config/urXX/joint_limits.yaml`` - If you'd like to further restrict the robot's joint limits,
   these limits can be modified there.
 - ``config/urXX/physical_parameters.yaml`` - Everything regarding physics simulation parameters
   (e.g. inertia poses and values) can be tuned here.
 - ``config/urXX/visual_parameters.yaml`` - Some users change certain visual aspects, e.g. replacing
   the cap on the wrist_3_link. This config file specifies which meshes (both, visual and collision)
   should be used.

The four configuration files have to be passed to ``ur_macro.xacro`` (more specific to the macro
defined in that file) which is done inside the ``ur.urdf.xacro``. Contents of the files are parsed
inside ``inc/ur_common.xacro``.

Arguments that have to be passed to the main ``ur.urdf.xacro`` file are:
 - kinematics_params - Filename to the ``default_kinematics.yaml`` (or equivalent specific kinematics) file
 - joint_limit_params - Filename to the ``joint_limits.yaml`` file
 - physical_params - Filename to the ``physical_parameters.yaml`` file
 - visual_params - Filename to the ``visual_params.yaml`` file

The launchfile ``launch/view_ur.launch.py`` abstracts these four parameters to one ``ur_type`` argument
which will basically replace the ``urXX`` part of the paths as shown in the picture above.

ros2_control integration
^^^^^^^^^^^^^^^^^^^^^^^^

`ros2_control <https://control.ros.org>`_ uses the robot_description in order to describe the
robot's control structure. Since this will vary for the respective control method (Driver for real
robot, Gazebo simulation, mock_hardware simulation, etc.) this is kept inside the respective control repository. This
repository provides macros for including the common control description (Joints with their
interfaces) as well as an example using mock hardware.

- ``urdf/ros2_control_mock_hardware.xacro`` - xacro providing the control description for
  mock_hardware.
- ``urdf/ur_mocked.urdf.xacro`` - an example for a controlled robot consisting of the kinematic
  description and the control description.
- ``urdf/inc/ur_joint_control.xacro`` - macro for describing the joints' control structure.
- ``urdf/inc/ur_sensors.xacro`` - macro for describing the robot's sensors for ros2_control

Creating your own description including this description
--------------------------------------------------------

In real-world applications you will most probably have a more complex description consisting of more objects than just the robot. It is recommended to create a separate ROS package containing this particular description. Inside this description you could also store your robot-specific kinematics parameters file.

See the `custom workcell example <https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/tree/main/my_robot_cell/my_robot_cell_description>`_ as an example for integrating this description in your own scene description.
