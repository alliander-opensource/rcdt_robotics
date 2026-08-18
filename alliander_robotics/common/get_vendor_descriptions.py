# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import os
import subprocess

VENDORS = ["franka", "husarion", "realsense"]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Clone description packages from the vendor repositories."
    )
    parser.add_argument(
        "-v", "--vendors", action="append", help="List of vendors.", required=False
    )
    args = parser.parse_args()

    vendors = args.vendors
    if not vendors:
        vendors = VENDORS
    else:
        for vendor in vendors:
            if vendor not in VENDORS:
                raise ValueError(f"Vendor '{vendor}' is not supported.")

    for vendor in vendors:
        match vendor:
            case "franka":
                cmd = f"""
                git clone -b 2.1.0 https://github.com/frankarobotics/franka_description.git src/franka_description \
                && echo 'GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:{os.getcwd()}/install/franka_description/share' >> /root/.bashrc
                """
            case "husarion":
                cmd = r"""
                git clone --depth=1 --filter=blob:none --sparse -b 2.3.1 https://github.com/husarion/husarion_ugv_ros.git src/husarion_ugv_ros \
                && cd src/husarion_ugv_ros && git sparse-checkout set husarion_ugv_description && cd ../..
                """
            case "realsense":
                cmd = r"""
                git clone --depth=1 --filter=blob:none --sparse -b 4.57.2 https://github.com/IntelRealSense/realsense-ros.git src/realsense-ros \
                && cd src/realsense-ros && git sparse-checkout set realsense2_description && cd ../..
                """
        subprocess.run(cmd, shell=True, check=True)
