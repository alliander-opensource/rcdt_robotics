<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# Platforms

This page describes the use of the different platforms we support.

## Franka

### Quick start

- The robot needs to be connected with it's control box.
- Flip the switch on the control box to start the robot.
- An ethernet cable should connect the control box with the network.
- By default, we connect the control box with the router of the Panther.
- The web interface can be reached on the statically assigned address: `10.15.20.4`.

In the web-interface, settings can be changed and the joints can be (un)locked. The mode can also be changed to the required FCI mode. It is also possible to unlock and activate the FCI entirely from the command line by dropping a `.env` file in your workspace root:

```text
# .env
FRANKA_HOSTNAME=your-hostname
FRANKA_USERNAME=your-username
FRANKA_PASSWORD=your-password
```

This functionality is based on [jk-ethz/franka\_lock\_unlock](https://github.com/jk-ethz/franka_lock_unlock).
We created a fork with robot-specific enhancements: [https://github.com/alliander-opensource/franka\_lock\_unlock.git](https://github.com/alliander-opensource/franka_lock_unlock.git)

## Panther

### Quick start

- Enable the battery (switch at the front of the robot).
- Start the robot (press red power button).
- Wait until the [E_STOP animation](https://husarion.com/manuals/panther/software/ros2/robot-management/#led-animations) is played
- Release hardware stop (rotate red emergency button if it was pressed).
- Start the Logitech gamepad:
  - press Logitech button.
  - press *mode* button if mode light is on (should be off).
  - put the switch at the back on *X*.
- Remove the E_STOP by *Left Trigger + A* on the gamepad.
- You can drive by pressing *Left Button* and use the two joysticks.
- You can enable the E_STOP by pressing *B*.
- See [this](https://husarion.com/manuals/panther/software/ros2/robot-management/#gamepad) for more information about gamepad control.

The robot can be shut down as follows:

- Shut down the robot (hold red button next to battery switch till it starts blinking).
- Wait until all lights are off.
- Disable the battery (switch at the front of the robot)

### Configuration

When the Panther is started, two WiFi networks (*Panther_<serial_number>* and *Panther_5G_<serial_number>*) should be available. One can connect with one of the WiFi networks or connect using a ethernet cable directly to the Teltonika. After connecting, it should be possible to ssh into all three computers.

**Teltonika RUTX11:**
\
This is an industrial router. The *Raspberry Pi 4* and *Lenovo ThinkStation P360* are connected to the *Teltonika* by Ethernet. Also the *Velodyne Lidar* and is connected to the *Teltonika* by Ethernet. A [combo antenna](https://teltonika-networks.com/products/accessories/antenna-options/combo-mimo-mobilegnsswi-fi-roof-sma-antenna) (the black dome) is also connected, which enables the *Teltonika* to obtain a GPS location.

**Raspberry Pi 4 :**
\
The *Raspberry Pi 4* is built in the front of the Panther and not directly accessible. Two Docker images are pre-installed: a [docker image](https://hub.docker.com/r/husarion/panther) of the [panther_ros](https://github.com/husarion/panther_ros) repository and a [docker image](https://hub.docker.com/r/husarion/joy2twist) of the [joy_to_twist](https://github.com/husarion/joy2twist) repository. Both images are started automatically when the robot starts, as do all docker images installed on the Pi. The first image runs all the required software to use the robot, like motor control and led control. The second image enables gamepad control with the Logitech gamepad shipped with the robot, when the USB receiver is connected to the USB port at the front of the robot.

We have also cloned the [nmea-gps-docker](https://github.com/husarion/nmea-gps-docker/tree/ros2) repository with a docker that enables use of GPS in ROS. This docker images gets started automatically as well when the the Panther starts. For more information about the use of this docker, see the Sensors section.

**Lenovo ThinkStation P360**
\
The *Lenovo ThinkStation P360* is a powerful computer, used to handle the camera stream. We can run our docker image on this built in computer.
