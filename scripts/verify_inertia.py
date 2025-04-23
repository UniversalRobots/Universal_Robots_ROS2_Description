#!/usr/bin/env python3

# Copyright (c) 2025 Universal Robots A/S
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

import sys
import yaml

import numpy as np

# Prints the inertia from a physical_parameters.yaml file in the same format as they would occur in
# the robot config file.


def main():
    filename = sys.argv[1]
    # Load the YAML file
    with open(filename) as file:
        data = yaml.safe_load(file)

        tensor = data["inertia_parameters"]["tensor"]
        for key, value in tensor.items():
            print(f"{key}:")
            a = np.zeros((3, 3))
            np.set_printoptions(suppress=True)
            a[0, 0] = value["ixx"]
            a[0, 1] = value["ixy"]
            a[0, 2] = value["ixz"]
            a[1, 0] = value["ixy"]
            a[1, 1] = value["iyy"]
            a[1, 2] = value["iyz"]
            a[2, 0] = value["ixz"]
            a[2, 1] = value["iyz"]
            a[2, 2] = value["izz"]

            print(a)


if __name__ == "__main__":
    main()
