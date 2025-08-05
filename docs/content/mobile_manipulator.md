<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# Mobile Manipulator

The **Mobile Manipulator** combines the Husarion Panther base with the Franka Emika arm in a fully modular stack. By treating each as a separate ROS 2 package—`rcdt_panther` and `rcdt_franka`, we can compose them under a new `rcdt_mobile_manipulator` package without duplicating launch or test logic. 

## Physical robot
The Franka Emika arm is powered through the battery of the panther and controlled through the build-in computer of the Panther. The controller is connected to the Panther's computer. 

1. First turn on the Panther and wait till the system is fully booted. 
2. Then run the following command to start the high-voltage system of the panther:

```bash
ros2 service call /panther/hardware/aux_power_enable std_srvs/srv/SetBool "{data: true}"
```
3. After that, turn on the Franka arm by flipping the button at the control box.
4. SSH in the build-in computer and start the combined system within the rcdt_robotics container with:

```bash
ros2 launch rcdt_mobile_manipulator mobile_manipulator.launch.py simulation:=False
``` 

## Simulation

Start the combined system with:

```bash
ros2 launch rcdt_mobile_manipulator mobile_manipulator.launch.py
```

![mobile-manipulator](../img/mobile_manipulator/rviz.png)

## Tests

By exposing the `rcdt_mobile_manipulator` launch description as a pytest fixture, you can **import and run** the existing Panther and Franka end-to-end tests against the combined system without any modification. Any improvements or fixes in the `rcdt_panther` and `rcdt_franka` test suites automatically apply when testing the mobile manipulator, keeping its codebase minimal and focused purely on orchestration.



