:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/rolling/doc/migration/kilted.rst

ur_description
^^^^^^^^^^^^^^

frame base_link_inertia removed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``base_link_inertia`` frame got removed as it is not really necessary, its name did not really
reflect its purpose and it lead to a lot of confusion.

With that change the visual and collision meshes of the base link are now directly attached to the
``base_link`` frame. This means that any default collision matrices of MoveIt support packages
have to be updated.

See `#282 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/pull/283>`_ for
details and `the frame documentation
<https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_description/doc/robot_frames.html>`_
for details.
