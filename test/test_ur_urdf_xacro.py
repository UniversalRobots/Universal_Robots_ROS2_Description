import os
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory


def test_ur_urdf_xacro():
    
    # Initialize Arguments
    ur_type = "ur3"
    safety_limits = "true"
    safety_pos_margin = "0.15"
    safety_k_position = "20"
    # General Arguments
    description_package = "ur_description"
    description_file = "ur.urdf.xacro"
    prefix = '""'

    joint_limit_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "joint_limits.yaml"
    )
    kinematics_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "default_kinematics.yaml"
    )
    physical_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "physical_parameters.yaml"
    )
    visual_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "visual_parameters.yaml"
    )

    description_file_path = os.path.join(
        get_package_share_directory(description_package),
        "urdf",
        description_file
    )

    (_, tmp_urdf_output_file) = tempfile.mkstemp(suffix=".urdf")

    # Compose `xacro` and `check_urdf` command
    xacro_command = (
        f"{shutil.which('xacro')}"
        f" {description_file_path}"
        f" joint_limit_params:={joint_limit_params}"
        f" kinematics_params:={kinematics_params}"
        f" physical_params:={physical_params}"
        f" visual_params:={visual_params}"
        f" safety_limits:={safety_limits}"
        f" safety_pos_margin:={safety_pos_margin}"
        f" safety_k_position:={safety_k_position}"
        f" name:={ur_type}"
        f" prefix:={prefix}"
        f" > {tmp_urdf_output_file}"
    )
    check_urdf_command = f"{shutil.which('check_urdf')} {tmp_urdf_output_file}"

    # Try to call processes but finally remove the temp file
    try:
        xacro_process = subprocess.run(
            xacro_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert xacro_process.returncode == 0, " --- XACRO command failed ---"

        check_urdf_process = subprocess.run(
            check_urdf_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert (
            check_urdf_process.returncode == 0
        ), "\n --- URDF check failed! --- \nYour xacro does not unfold into a proper urdf robot description. Please check your xacro file."

    finally:
        os.remove(tmp_urdf_output_file)


if __name__ == '__main__':
    test_ur_urdf_xacro()
