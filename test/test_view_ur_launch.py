import os
import launch_testing
import pytest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

# Executes the given launch file and checks if all nodes can be started
@pytest.mark.rostest
def generate_test_description():

    pkg_share = get_package_share_directory("ur_description")
    launch_dir = os.path.join(pkg_share, "launch")

    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur_description"),
                "launch/view_ur.launch.py")),
        launch_arguments={"ur_type": "ur3"}.items(),
    )

    return LaunchDescription([
        launch_include,
        launch_testing.actions.ReadyToTest()
    ])
