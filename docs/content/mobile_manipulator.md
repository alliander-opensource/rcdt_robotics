<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# Mobile Manipulator

The **Mobile Manipulator** combines the Husarion Panther base with the Franka Emika arm in a fully modular stack. By treating each as a separate ROS 2 package—`rcdt_panther` and `rcdt_franka`—we can compose them under a new `rcdt_mobile_manipulator` package without duplicating launch or test logic.

## Launch

Start the combined system with:

```bash
ros2 launch rcdt_mobile_manipulator mobile_manipulator.launch.py
```

![mobile-manipulator](../img/mobile_manipulator/rviz.png)

## Tests

By exposing the `rcdt_mobile_manipulator` launch description as a pytest fixture, you can **import and run** the existing Panther and Franka end-to-end tests against the combined system without any modification. Any improvements or fixes in the `rcdt_panther` and `rcdt_franka` test suites automatically apply when testing the mobile manipulator, keeping its codebase minimal and focused purely on orchestration.



