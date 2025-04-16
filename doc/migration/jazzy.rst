:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/rolling/doc/migration/jazzy.rst

ur_description
^^^^^^^^^^^^^^

ros2_control xacro tag moved to driver package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The description package no longer adds the ``ros2_control`` tag to the robot's URDF. This is done
in the driver / simulation repos now. The description package still provides macros for the
system-independent parts such as joint configurations and an example using mock_hardware with the
robot.

Enforce absolute paths in launchfiles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``view_ur.launch.py`` launchfile now expects an absolute path for the description file and RViz
config file. Before it was expecting a ``description_package`` and a ``description_file`` argument
with a relative path to the package.
This way, users can provide their own description file and / or RViz config without the need to
replicate the complete package structure.

Absolute paths can still be generated dynamically using a package + relative path structure inside
other launchfiles or by using ``ros2 pkg prefix`` on the command line.
