<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# Franka

This page gives some background of the usage of our Franka-related packages.  
All software is based on the [Franka Emika Research](https://franka.de/products), based on the [Franka Control Interface](https://frankaemika.github.io/docs/overview.html)

## Quickstart robot in simulation

To start the simulation, run in 2 separate terminals:  

```bash
ros2 launch rcdt_franka franka.launch.py realsense:=True load_gazebo_ui:=True
```

```bash
ros2 run rosboard rosboard_node
```

This launches the robot in a gazebo world with a brick place on it.  
It also allows the inspection of sensor output on [localhost:8888](http://localhost:8888)
The robot can be controled using a gamepad, or using moveit MotionPlanning in rviz under:  
`add>moveit_ros_visualization>MotionPanning`

In order to run the brick pickup demo, open up 2 more terminals and run:

```bash
ros2 launch rcdt_detection detection_services.launch.py
```

```bash
pyflow
```

The relevant pygraph is available under:  
`/home/rcdt/rcdt_robotics/pyflow/graphs/pick_brick.pygraph`

## Quickstart physical robot

When you have a properly configured physical robot and a realsense at your disposal. In 2 separate terminals, run:

```bash
ros2 launch rcdt_franka franka.launch.py simulation:=False realsense:=True
```

```bash
ros2 run rosboard rosboard_node
```

Running the brick pickup demo can be done in the same way.


## Setup & FCI activation of physical robot

### Programmatic unlock via `.env`

You can unlock the robot automatically without using the browser by setting environment variables in a `.env` file at the root of your workspace:

```dotenv
# .env
FRANKA_HOSTNAME=your-hostname
FRANKA_USERNAME=your-username
FRANKA_PASSWORD=your-password
```

The robot will automatically unlock and activate the FCI when you run the launch command.

### Manual unlock via web interface

You can also manually lock the robot via the web interface, plug an Ethernet cable from your laptop into the control box’s LAN port (this is not the LAN port on the arm itself).  Additionally, change your Ethernet IPv4 settings to **Manual** and set the following:

```text
Address: 172.16.0.1
Netmask: 255.255.255.0
```

![Network Setup](../img/franka/network.png)

You may also want to consider making this a separate profile for future convenience when moving to different networks.

If the robot's power switch is turned on, you should now be able to go to [https://172.16.0.2](https://172.16.0.2).
Fill in the username and password, and click **Unlock**. After that, click **My Franka Robot → Activate FCI**. The robot is now ready to be controlled by the user, and the commands in Quickstart can be run.

---TODO: screenshot here----
