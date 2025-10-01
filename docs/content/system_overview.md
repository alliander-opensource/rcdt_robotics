<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# System Overview

The following diagram gives a very simplified overview of the current state of our robot system:

:::{mermaid} ../diagrams/system_overview.mmd
:::

You can see two robots (`franka` and `panther`) and four sensors (`realsense`, `zed`, `teltonika` and `velodyne`). Note that the output of the teltonika (gps) and the output of the velodyne (pointcloud) are used in `nav2` to navigate the panther. Output of the depth camera's are not used at the moment. There are three ways to control the robot:

:::{mermaid} ../diagrams/node_definitions.mmd
:::

*joystick*:
\
This joystick contains a `joystick_topic_manager` to switch the control between franka and panther. A `joy_to_twist` node converts the joy message to a twist message. For panther, this twist message can be directly send to the drive controller. For franka, `moveit_servo` is used to calculate the required joint command that results in the desired twist command of the end effector.

*moveit_manager*:
\
For robot arms, we created a `moveit_manager` node as a layer above the `moveit_ros_move_group` node. This manager provides functionality like moving to a default configuration or moving the end effector to a given pose.

*nav2*
\
For robot vehicles, we use `nav2`, a set of nodes that provide functionality like navigating to a pose or following waypoints.

At the moment, we can use the joystick, or functionality of the moveit_manager or nav2 to control the robots.

**vendor elements**
\
Note that the elements in <font color="purple">purple</font> are provided by the vendor of the robot or sensor. We expect some basic functionality, like a controller for a robot and the data output of sensor, provided by the vendor of the product. We only make small adjustments or support for simulation when required.

**default ROS elements**
\
Note that the elements in <font color="blue">blue</font> are default in ROS and not created by us.

**namespaces**
\
Note that each robot and sensor *lives* in it's own namespace. This enables us to add as many robots and sensors to our system as we like, even when they are the same. For example: if we would have a second Franka arm, we can define the namespaces `franka1` and `franka2`.

**modularity**
\
With this setup we focus on the modularity of the system. We can add sensors and robots if we like and reuse nodes that we already developed. If we add another robot arm, we expect the vendor to provide the correct controller nodes. We only need to define the related parameters and should be able to use it again with our Moveit Manager and the default Moveit nodes provided in ROS. And if we add another robot vehicle, we can still use our current Nav2 setup, only requiring the correct definition of the related parameters.

**central controller**
\
At the moment, a human is still the central controller in our system. A human can control both the robots using the joystick or via service/action calls to the Moveit Manager or Nav2. Our goal is to get the human out of the loop.
