<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# Franka

This page gives information about usage of the Franka Research 3.

## Physical robot

### Quick start

The physical robot can be started with the following steps:

- Start the robot (flip the button at the control box)
- Connect your laptop with the control box's LAN port using an ethernet cable.
- Make sure to use these ethernet settings:

Address: `172.16.0.1`\
Netmask: `255.255.255.0`

![network](../img/franka/network.png)

You may also want to consider making this a separate profile for future convenience.

- Go to [https://172.16.0.2](https://172.16.0.2), Franka Desk should now be reachable.
- Login and click *unlock* to unlock the joints.
- Click *My Franka Robot > Activate FCI* to activate FCI, the led's should become green.
- Launch the franka launch file with *simulation=False*:

```bash
ros2 launch rcdt_franka franka.launch.py simulation:=False
```

This should launch Rviz and show the state of the robot:

![franka](../img/franka/franka.png)

You can control the robot using a connected gamepad or using the *MotionPlanning* plugin in Rviz.

## Simulation

You can also start a simulation, without requiring a real Franka arm. Use the same launch file, but this time without the *simulation* flag:

```bash
ros2 launch rcdt_franka franka.launch.py
```
