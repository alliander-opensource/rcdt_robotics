# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from glob import glob
from os.path import basename
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_lib_path


def generate_launch_description() -> LaunchDescription:
    nodes = []
    executables_directory = get_lib_path("rcdt_detection")
    executables = glob(executables_directory + "/*.py")
    for executable in executables:
        path = Path(executable)
        if path.is_symlink() and not path.exists():
            continue
        nodes.append(Node(package="rcdt_detection", executable=basename(executable)))
    return LaunchDescription(nodes)
