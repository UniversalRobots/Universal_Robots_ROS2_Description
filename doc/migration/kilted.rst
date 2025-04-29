:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/rolling/doc/migration/kilted.rst

ur_description
^^^^^^^^^^^^^^

Radius definitions removed
~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``shoulder_radius: ...`` and alike for the other joints got removed from the
``physical_parameters.yaml`` files. Alongside parsing these got removed from
``urdf/inc/ur_common.xacro``. Using the new parameter files with the old parser (if that git copied
to a modified package) will fail.

All mesh offsets are now part of the visual_parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``physical_parameters`` did contain the two mesh offsets for the shoulder and elbow. These got
now moved to the ``visual_parameters.yaml`` files.

Mesh offsets defined with all 6 DOF
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``visual_parameters.yaml`` files now contain the mesh offsets for all 6 dimensions. The URDF
expects this structure of the config files. So, the URDF won't accept any old visual parameter
config files. They will have to be updated to contain the full ``mesh_offset`` entries.
