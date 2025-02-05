<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# PyFlow

This page gives some background of the usage of our Franka-related packages.  
All software is based on the [Franka Emika Research](https://franka.de/products), based on the [Franka Control Interface](https://frankaemika.github.io/docs/overview.html)

## Quickstart robot in simulation

To start the simulation, run in 2 separate terminals:  

```bash
ros2 launch rcdt_franka franka.launch.py realsense:=True load_gazebo_ui:=True

ros2 run rosboard rosboard_node
```
This launches the robot in a gazebo world with a brick place on it.  
It also allows the inspection of sensor output on [localhost:8888](localhost:8888)
The robot can be controled using a gamepad, or using moveit MotionPlanning in rviz under:  
add>moveit_ros_visualization>MotionPanning.

In order to run the brick pickup demo, open up 2 more terminals and run:
```bash
ros2 launch rcdt_detection detection_services.launch.py

pyflow
```
The relevant pygraph is available under:  
/home/rcdt/rcdt_robotics/pyflow/graphs/pick_brick.pygraph
## Quickstart physical robot

When you have a properly configured physical robot and a realsense at your disposal In 2 separate terminals, run:
```bash
ros2 launch rcdt_franka franka.launch.py simulation:=False realsense:=True

ros2 run rosboard rosboard_node
```
Running the brick pickup demo can be done in the same way

## setup & FCI activation of pysical robot

First plug an ehternet cable into your laptop, and connect it to the control box's LAN port (this is not the LAN port on the arm itself).  
Additionally, change your ethernet ipv4 settings to Manual and set the following:  
Address: 172.16.0.1  
Netmask: 255.255.255.0  

![pyflow](../img/franka/network.png)

You may also want to consider making this a separate profile for future convenience when moving to different networks.

If the robot's power switch is turned on, you should now be able to go to [https://172.16.0.1](https://172.16.0.1).  
Fill in the username and password, and click unlock.  
After that, click My Franka Robot> Activate FCI. The robot is now ready to be controlled by the user, and the commands in quickstart can be run.

---TODO: screenshot here----