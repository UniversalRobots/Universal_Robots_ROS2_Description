^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.11 (2025-03-17)
-------------------
* Fix UR3 mesh positioning (backport of `#258 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/258>`_) (`#259 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/259>`_)
* Auto-update pre-commit hooks (backport of `#254 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/254>`_) (`#255 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/255>`_)
* Auto-update pre-commit hooks (backport `#252 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/252>`_) (`#253 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/253>`_)
* Add missing gpio interfaces for force_mode and freedrive_mode (`#251 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/251>`_)
* Auto-update pre-commit hooks (`#250 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/250>`_)
* Contributors: Felix Exner

2.1.10 (2025-01-23)
-------------------
* Fix ur20 upperarm texture (backport `#244 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/244>`_) (`#246 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/246>`_)
* Removing Blender Lighting and Camera information from visual DAE files (backport `#243 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/243>`_) (`#245 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/245>`_)
* Auto-update pre-commit hooks (backport of `#241 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/241>`_)
* Update package maintainers (backport of `#238 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/238>`_)
* Remove Iron workflows and from README (backport of `#230 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/230>`_)
* Contributors: Felix Exner, Shaurya Kumar

2.1.9 (2024-12-04)
------------------
* Assure the description is loaded as string (backport `#229 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/229>`_)
* Added ground plane to URDF for simulators (`#226 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/226>`_)
* Contributors: Vincenzo Di Pentima, Felix Exner

2.1.8 (2024-10-28)
------------------
* Add analog_output_domain_cmd command interface (baclport of `#219 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/219>`_)
* Add a sensor for the TCP pose (backport of `#197 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/197>`_)
* Add missing state interfaces for get_version service (backport of `#216 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/216>`_)
* Ur3 infinite wrist (backport of `#196 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/196>`_)
* Update dynamic properties (backport of `#195 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/195>`_)
* Contributors: mergify[bot], Felix Exner, Rune Søe-Knudsen

2.1.7 (2024-09-10)
------------------
* Fix masses of robot links (backport of `#187 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/187>`_)
* Contributors: Felix Exner

2.1.6 (2024-08-09)
------------------
* Fixed typo in README.md (`#176 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/176>`_)
* Added dynamics tag when using mock_components/GenericSystem (backport of `#175 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/175>`_)
* Auto-update pre-commit hooks (`#171 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/171>`_) (`#172 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/172>`_)
* Remove ros2_control limit params (`#168 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/168>`_)
* Add Jazzy to the README (`#163 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/163>`_)
* Contributors: Filippo Bosi, Niccolo, Felix Exner

2.1.5 (2024-04-25)
------------------
* Fix multi-line strings in DeclareLaunchArgument ( backport of `#140 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/140>`_) (`#153 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/153>`_)
* Fix default calibration file for UR30 (`#148 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/148>`_)
* Contributors: Matthijs van der Burgh, Felix Exner

2.1.4 (2024-04-04)
------------------
* Update Graphical Documentation license to version 1.01 (`#143 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/143>`_)
* Add UR30 model (`#142 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/142>`_)
* Make sure the UR5 models are actually standing on the ground (`#136 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/pull/136>`_)
* Contributors: Felix Exner, RobertWilbrandt, Vincenzo Di Pentima

2.1.3 (2023-12-18)
------------------
* Make ros2_control tag generation optional in macro (`#121 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/121>`_)
* Contributors: Felix Exner (fexner)

2.1.2 (2023-11-17)
------------------
* Add license comment to package.xml (`#107 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/107>`_)
* License update for README (`#108 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/108>`_)
* Default to non_blocking_read=true (`#115 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/115>`_)
* added possibility to change reverse_port, script_sender_port and trajectory_port (`#105 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/105>`_) (`#106 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/106>`_)
* Update README regarding distribution branches (`#80 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/80>`_) (`#86 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/86>`_)
* Contributors: Felix Exner, Rune Søe-Knudsen, mergify[bot]

2.1.1 (2023-09-08)
------------------
* Update the joint limits for UR20 (`#99 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/99>`_)
* UR20 description and meshes (`#94 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/94>`_)
  The UR20 meshes are added under Universal Robots A/S’
  Terms and Conditions for Use of Graphical Documentation
  Co-authored-by: Rune Søe-Knudsen <41109954+urrsk@users.noreply.github.com>
* Revert "Switch fake to mock for ros2_control updates (`#77 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/77>`_)"
* Switch fake to mock for ros2_control updates (`#77 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/77>`_)
* CI: Add iron workflow (`#64 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/64>`_)
* Contributors: Felix Exner, Sebastian Castro, Rune Søe-Knudsen

2.1.0 (2023-06-01)
------------------
* added missing handback interface - ros2control mock interface won't work otherwise (`#68 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/68>`_)
  Co-authored-by: Lennart Nachtigall <lennart.nachtigall@sci-mo.de>
* remove ticks from tf_prefix (`#60 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/60>`_)
  Co-authored-by: Lennart Nachtigall <lennart.nachtigall@sci-mo.de>
* Replace duplicated ``prefix`` parameter with ``tf_prefix``
* Whitespace fixes
* Update pre-commit workflows to current versions
* This commits adds additional configuration fields which are needed for multiarm support: (`#47 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/47>`_)
  - Added trajectory_port        - Port needed for the trajectory sending interface
  - Added non_blocking_read      - Takes control of the update rate from ur interface by immediately returning from the read method
  - Added keep_alive_count field - Configures the amount of allowed reading timeouts on the robot side
  Additionally it adds the ${prefix} argument for the gpios and the force torque sensor in the ur.ros2_control.xacro file
  Co-authored-by: Lennart Nachtigall <firesurfer@firesurfer.de>
* Set the default tool voltage in the description to 0 (`#41 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/41>`_)
  I am not sure whether this will actually affect something, as I don't think
  we actually set the value initially, but it still makes sense to keep the
  default tool voltage at 0 to emphasize that by default, this will not be
  set higher.
* Run prerelease tests on current distros (`#44 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/44>`_)
* Contributors: Felix Exner, Felix Exner (fexner), Lennart Nachtigall

2.0.1 (2022-11-08)
------------------
* Add tool voltage and zero ft sensor to command interface (`#38 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/38>`_)
  Added reverse ip and script command interface port as parameters
* use xacro.load_yaml in favor of deprecated version (`#43 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/43>`_)
  Co-authored-by: aditya <aditya@nimble.ai>
* Use mock_components instead of fake_components (`#37 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/37>`_)
  This has been renamed in ros2_control hardware_interface.
* Prepare for branching out galactic (`#39 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/issues/39>`_)
  * Add Humble to README and workflows
  * Use galactic branch for galactic stuff
* Contributors: Abishalini Sivaraman, Aditya Agarwal, Felix Exner, Mads Holm Peters

2.0.0 (2022-03-17)
------------------
* Migrated the description to ROS2
* Added support for Gazebo and Ignition
* Added ROS2_control definitions
* Contributors: AndyZe, Denis Stogl, Denis Štogl, Felix Exner, John Morris, Jorge Nicho, Lovro, Lukas Sackewitz, Marvin Große Besselmann, Robert Wilbrandt, Tirine, Vatan Aksoy Tezer, livanov93, urmahp

1.2.7 (2019-11-23)
------------------

1.2.6 (2019-11-19)
------------------
* Add optional safety_controller tags to all joints in xacro macros (`#437 <https://github.com/ros-industrial/universal_robot/issues/437>`_)
* Migrated all package.xml files to format=2 (`#439 <https://github.com/ros-industrial/universal_robot/issues/439>`_)
* Corrected dimensions and positions of inertias (`#426 <https://github.com/ros-industrial/universal_robot/issues/426>`_)
* Add description view launch files for all descriptions to easily check them (`#435 <https://github.com/ros-industrial/universal_robot/issues/435>`_)
* Contributors: Felix Mauch, JeremyZoss, Miguel Prada, Qiang Qiu, gavanderhoorn

1.2.5 (2019-04-05)
------------------
* Add transmission_hw_interface to UR xacro and expose everywhere (`#392 <https://github.com/ros-industrial/universal_robot/issues/392>`_)
* Update maintainer listing: add Miguel (`#410 <https://github.com/ros-industrial/universal_robot/issues/410>`_)
* Updated xacro namespace.
* Update maintainer and author information.
* Updated mesh ambience so the model isn't so dark in Gazebo
* Fix overlapping variable names between robot definition files (`#356 <https://github.com/ros-industrial/universal_robot/issues/356>`_)
* Improve meshes shading (`#233 <https://github.com/ros-industrial/universal_robot/issues/233>`_)
* Added run_depend for xacro
* Using the 'doc' attribute on 'arg' elements.
* Enable self collision in gazebo
* Contributors: Dave Niewinski, Felix von Drigalski, Harsh Deshpande, Joe, Marcel Schnirring, Miguel Prada, MonteroJJ, ipa-fxm

1.2.1 (2018-01-06)
------------------
* Merge pull request `#329 <https://github.com//ros-industrial/universal_robot/issues/329>`_ from tecnalia-medical-robotics/joint_limits
  Homogenize xacro macro arguments.
* Merge pull request `#332 <https://github.com//ros-industrial/universal_robot/issues/332>`_ from davetcoleman/kinetic_hw_iface_warning
  Remove UR3 ROS Control Hardware Interface warning
* Remove UR3 ROS Control Hardware Interface warning
* Extend changes to '_robot.urdf.xacro' variants as well.
* Homogenize xacro macro arguments.
  Joint limits for the limited version could be set using arguments for the UR10
  but not for the UR3 and UR5. Same lower and upper limit arguments are added to
  the UR3 and UR5 xacro macros.
* Fix elbow joint limits (`#268 <https://github.com//ros-industrial/universal_robot/issues/268>`_)
* Remove warning 'redefining global property: pi' (Jade+) (`#315 <https://github.com//ros-industrial/universal_robot/issues/315>`_)
* Contributors: Beatriz Leon, Dave Coleman, Felix Messmer, Miguel Prada

1.2.0 (2017-08-04)
------------------

1.1.9 (2017-01-02)
------------------
* reintroduce 'pi', unbrake dependent xacros.
* use '--inorder' to trigger use of jade+ xacro on Indigo.
* Contributors: gavanderhoorn

1.1.8 (2016-12-30)
------------------
* all: update maintainers.
* Contributors: gavanderhoorn

1.1.7 (2016-12-29)
------------------
* Fix xacro warnings in Jade (`ros1#251 <https://github.com/ros-industrial/universal_robot/issues/251>`_)
* added default values to xacro macro
* tested joint limits modification
* Contributors: Dave Coleman, G.A. vd. Hoorn, philip 14.04

1.1.6 (2016-04-01)
------------------
* unify mesh names
* add color to avoid default color 'red' for collision meshes
* use correct DH parameter + colored meshes
* introducing urdf for ur3 - first draft
* unify common xacro files
* remove obsolete urdf files
* description: add '_joint' suffix to newly introduced joint tags.
  This is more in-line with naming of existing joint tags.
* description: add ROS-I base and tool0 frames. Fix `#49 <https://github.com/ros-industrial/universal_robot/issues/49>`_ and `#95 <https://github.com/ros-industrial/universal_robot/issues/95>`_.
  Note that 'base' is essentially 'base_link' but rotated by 180
  degrees over the Z-axis. This is necessary as the visual and
  collision geometries appear to also have their origins rotated
  180 degrees wrt the real robot.
  'tool0' is similar to 'ee_link', but with its orientation such
  that it coincides with an all-zeros TCP setting on the UR
  controller. Users are expected to attach their own TCP frames
  to this frame, instead of updating it (see also [1]).
  [1] http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages#Standardised_links\_.2BAC8_frames
* description: minor whitespace cleanup of UR5 & 10 xacros.
* regenerate urdf files
* use PositionJointInterface as hardwareInterface in transmissions - affects simulation only
* Contributors: gavanderhoorn, ipa-fxm

1.0.2 (2014-03-31)
------------------

1.0.1 (2014-03-31)
------------------
* changes due to file renaming
* generate urdfs from latest xacros
* file renaming
* adapt launch files in order to be able to use normal/limited xacro
* fixed typo in limits
* add joint_limited urdf.xacros for both robots
* (re-)add ee_link for both robots
* updates for latest gazebo under hydro
* remove ee_link - as in ur10
* use same xacro params as ur10
* use new transmission interfaces
* update xml namespaces for hydro
* remove obsolete urdf file
* remove obsolete urdf file
* Contributors: ipa-fxm

* Update ur10.urdf.xacro
  Corrected UR10's urdf to faithfully represent joint effort thresholds, velocity limits, and dynamics parameters.
* Update ur5.urdf.xacro
  Corrected effort thresholds and friction values for UR5 urdf.
* added corrected mesh file
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Corrected warning on xacro-files in hydro.
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Updated to catkin.  ur_driver's files were added to nested Python directory for including in other packages.
* fixed name of ur5 transmissions
* patched gazebo.urdf.xacro to be compatible with gazebo 1.5
* fixed copy&paste error (?)
* prefix versions of gazebo and transmission macros
* Added joint limited urdf and associated moveit package.  The joint limited package is friendlier to the default KLD IK solution
* Added ur5 moveit library.  The Kinematics used by the ur5 move it library is unreliable and should be replaced with the ur_kinematics
* Updated urdf files use collision/visual models.
* Reorganized meshes to include both collision and visual messhes (like other ROS-I robots).  Modified urdf xacro to include new models.  Removed extra robot pedestal link from urdf (urdfs should only include the robot itself).
* minor changes on ur5 xacro files
* Removed extra stl files and fixed indentions
* Renamed packages and new groovy version
* Added ur10 and renamed packages
* Contributors: Denis Štogl, IPR-SR2, Kelsey, Mathias Lüdtke, Shaun Edwards, ipa-nhg, jrgnicho, kphawkins, robot
