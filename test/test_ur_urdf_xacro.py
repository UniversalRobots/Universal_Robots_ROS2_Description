# Copyright (c) 2022 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Lukas Sackewitz

import os
import pytest
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


@pytest.mark.parametrize(
    "ur_type",
    [
        "ur3",
        "ur5",
        "ur10",
        "ur3e",
        "ur5e",
        "ur7e",
        "ur10e",
        "ur12e",
        "ur16e",
        "ur8long",
        "ur15",
        "ur18",
        "ur20",
        "ur30",
    ],
)
@pytest.mark.parametrize("description_file", ["ur.urdf.xacro", "ur_mocked.urdf.xacro"])
@pytest.mark.parametrize("prefix", ["", "my_ur_"])
def test_ur_urdf_xacro(ur_type, description_file, prefix):
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
        get_package_share_directory(description_package), "config", ur_type, "joint_limits.yaml"
    )
    kinematics_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "default_kinematics.yaml",
    )
    physical_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "physical_parameters.yaml",
    )
    visual_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "visual_parameters.yaml",
    )

    description_file_path = os.path.join(
        get_package_share_directory(description_package), "urdf", description_file
    )

    _, tmp_urdf_output_file = tempfile.mkstemp(suffix=".urdf")

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


@pytest.mark.parametrize(
    ("argument", "expected"),
    [
        (None, "True"),
        ("true", "True"),
        ("false", "False"),
    ],
)
def test_use_currents_as_efforts_hardware_parameter(argument, expected):
    description_file_path = os.path.join(
        get_package_share_directory("ur_description"), "urdf", "ur.urdf.xacro"
    )
    command = [
        shutil.which("xacro"),
        description_file_path,
        "name:=ur5e",
        "ur_type:=ur5e",
    ]
    if argument is not None:
        command.append(f"use_currents_as_efforts:={argument}")

    result = subprocess.run(command, capture_output=True, check=True, text=True)
    robot = ET.fromstring(result.stdout)
    parameter = robot.find(".//ros2_control/hardware/param[@name='use_currents_as_efforts']")

    assert parameter is not None
    assert parameter.text == expected


def _generate_urdf(ur_type, physical_params=None, visual_params=None):
    description_package = "ur_description"
    package_path = get_package_share_directory(description_package)
    description_file_path = os.path.join(package_path, "urdf", "ur.urdf.xacro")
    physical_params = physical_params or os.path.join(
        package_path, "config", ur_type, "physical_parameters.yaml"
    )
    visual_params = visual_params or os.path.join(
        package_path, "config", ur_type, "visual_parameters.yaml"
    )
    _, tmp_urdf_output_file = tempfile.mkstemp(suffix=".urdf")

    xacro_command = (
        f"{shutil.which('xacro')}"
        f" {description_file_path}"
        f" ur_type:={ur_type}"
        f" name:={ur_type}"
        f" physical_params:={physical_params}"
        f" visual_params:={visual_params}"
        f" > {tmp_urdf_output_file}"
    )
    check_urdf_command = f"{shutil.which('check_urdf')} {tmp_urdf_output_file}"

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

        with open(tmp_urdf_output_file, encoding="utf-8") as urdf_file:
            return urdf_file.read()
    finally:
        os.remove(tmp_urdf_output_file)


def _upper_arm_visual_origin(urdf_content):
    robot = ET.fromstring(urdf_content)
    return robot.find("./link[@name='upper_arm_link']/visual/origin")


def test_legacy_mesh_offset_format():
    origin = _upper_arm_visual_origin(_generate_urdf("ur10"))

    assert origin is not None
    assert [float(value) for value in origin.get("xyz").split()] == pytest.approx(
        [0.0, 0.0, 0.220941]
    )
    assert [float(value) for value in origin.get("rpy").split()] == pytest.approx(
        [1.5707963267948966, 0.0, -1.5707963267948966]
    )


def test_new_mesh_offset_format(tmp_path):
    package_path = get_package_share_directory("ur_description")
    config_path = os.path.join(package_path, "config", "ur10")

    with open(os.path.join(config_path, "physical_parameters.yaml"), encoding="utf-8") as source:
        physical_lines = source.readlines()

    filtered_physical_lines = []
    skipping_offsets = False
    for line in physical_lines:
        if line.startswith("offsets:"):
            skipping_offsets = True
            continue
        if skipping_offsets and line.startswith("  "):
            continue
        skipping_offsets = False
        if "_radius:" not in line:
            filtered_physical_lines.append(line)

    physical_params = tmp_path / "physical_parameters.yaml"
    physical_params.write_text("".join(filtered_physical_lines), encoding="utf-8")

    with open(os.path.join(config_path, "visual_parameters.yaml"), encoding="utf-8") as source:
        visual_content = source.read()

    mesh_offset = """\
    mesh_offset:
      x: 0.1
      y: 0.2
      z: 0.3
      roll: 0.4
      pitch: 0.5
      yaw: 0.6

"""
    visual_content = visual_content.replace("\n  forearm:\n", f"\n{mesh_offset}  forearm:\n")
    visual_params = tmp_path / "visual_parameters.yaml"
    visual_params.write_text(visual_content, encoding="utf-8")

    origin = _upper_arm_visual_origin(
        _generate_urdf("ur10", str(physical_params), str(visual_params))
    )

    assert origin is not None
    assert [float(value) for value in origin.get("xyz").split()] == pytest.approx([0.1, 0.2, 0.3])
    assert [float(value) for value in origin.get("rpy").split()] == pytest.approx([0.4, 0.5, 0.6])


if __name__ == "__main__":
    test_ur_urdf_xacro()
