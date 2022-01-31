import os
import pytest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

# tests entire launch file
@pytest.mark.rostest
def generate_test_description():
    
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur_description"),
                "launch/view_ur.launch.py")),
        launch_arguments={"ur_type": "ur3"}.items(),
    )

    return LaunchDescription([
        launch_include,
        ReadyToTest()
    ])
