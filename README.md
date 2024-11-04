<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# RCDT Robotics

This repository gives an overview of the robots from RCDT and the different repositories developed for these robots.

**Robots:**

| ![drawing](img/fr3.jpg) | ![drawing](img/hp.jpg) |
| :---------------------: | :--------------------: |
|    Franka Research 3    |    Husarion Panther    |

**General repositories:**

- [rcdt_docker](https://github.com/alliander-opensource/rcdt_docker)
- [rcdt_linting](https://github.com/alliander-opensource/rcdt_linting)
- [rcdt_utilities](https://github.com/alliander-opensource/rcdt_utilities)
- [rcdt_detection](https://github.com/alliander-opensource/rcdt_detection)

**Robot specific repositories:**

- [rcdt_franka](https://github.com/alliander-opensource/rcdt_franka)
- [rcdt_panther](https://github.com/alliander-opensource/rcdt_panther)
- [rcdt_mobile_manipulator](https://github.com/alliander-opensource/rcdt_mobile_manipulator)

**Forked repositories:**

- [franka_description](https://github.com/alliander-opensource/franka_description)

## General repositories

The following repositories are general repositories, used for all the robots.

**rcdt_docker:**

Contains scripts to generate different docker images for the different robots. This simplifies the process of installing the software requirements for every robot and makes switching between different robot systems easier.

**rcdt_linting:**

Contains github workflows, used by the other repositories, used for automatic checks on code format and quality.

**rcdt_utilities:**

Contains ROS2-based utility functions that can be used for ROS2 software development for any robot. This avoids duplication of often repeated code between different robotic specific repositories.

**rcdt_detection:**

Contains ROS2-based detection packages.

## Robot specific repositories

The following repositories contain software developed for a specific robot:

**rcdt_franka:**

Contains the RCDT-developed software for the Franka Research 3.

**rcdt_panther:**

Contains the RCDT-developed software for the Husarion Panther.

**rcdt_mobile_manipulator:**

Contains RCDT-developed software for mobile manipulators. So far, the only supported mobile manipulator is a combination of the Husarion Panther base with a Franka Research 3 arm. Therefore, this repository depends on *rcdt_franka* and *rcdt_panther*.

## Forked repositories

The following repositories are forked and adapted for our needs:

**franka_description:**

Contains the description of the franka robot. We made adaptions to:

- support gripper usage in simulation
- enable usage in combination with a driving base

## License

This project is licensed under the Apache License Version 2.0 - see [licence](./LICENSES/Apache-2.0.txt) for details.

## Contributing

Please read [CODE_OF_CONDUCT](CODE_OF_CONDUCT.md), [CONTRIBUTING](CONTRIBUTING.md), and [PROJECT GOVERNANCE](PROJECT_GOVERNANCE.md) for details on the process for submitting pull requests to us.
