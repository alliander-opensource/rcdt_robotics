<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# RCDT Robotics

This repository is used for development of robotics in the Alliander Research Center for Digital Technologies (RCDT). Currently, RCDT has two robots we use for development:

| ![drawing](img/fr3.jpg) | ![drawing](img/hp.jpg) |
| :---------------------: | :--------------------: |
|    Franka Research 3    |    Husarion Panther    |

## Clone this repository

To use this repository, first clone it including the submodules:

`git clone --recurse-submodules https://github.com/alliander-opensource/rcdt_robotics.git`

## Install docker

This repository contains docker files to simplify the installation process of the required software. To install docker itself, please check: [docs/install_docker](docs/install_docker.md).

## git lfs

This repository uses git lfs for larger files, in particular for 3D simulation assets. To install git LFS please check: [git lfs](https://git-lfs.com/).

## Build/run an image

To build and run one of our images, use the run script in root of the repository and follow the instructions:

`. run`

The selected image will build and run.

## ROS2 packages

The ROS2 packages, developed for the robots, can be found in `ros2_ws/src`. While using the docker image, one can build the packages inside the ros2_ws, using colcon:

`colcon build --symlink-install`

Do not forget to source afterwards:

`source /home/rcdt/ros2_ws/install/setup.bash`

You can also automate this by adding the source command to the `.bashrc.personal` file, in the root of this repository.

You should now be able to use the ROS2 packages. Check the package specific README files for further instructions.

## License

This project is licensed under the Apache License Version 2.0 - see [licence](./LICENSES/Apache-2.0.txt) for details.

## Contributing

Please read [CODE_OF_CONDUCT](CODE_OF_CONDUCT.md), [CONTRIBUTING](CONTRIBUTING.md), and [PROJECT GOVERNANCE](PROJECT_GOVERNANCE.md) for details on the process for submitting pull requests to us.
