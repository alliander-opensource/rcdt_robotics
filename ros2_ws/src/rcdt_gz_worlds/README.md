<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# RCDT gz worlds

This package provides worlds to use in gazebo, along with associated resources like models that populate these worlds.

## Usage

You can launch the gazebo world with:

```
ros2 launch rcdt_gz_worlds world.launch.py
```

## Available worlds


### Empty world fit for camera (empty_camera.sdf)

![drawing](img/empty_camera.jpeg)  
An empty world, with the plugins needed for simulating sensors.

### Table with single brick (table_with_1_brick.sdf)

![drawing](img/table_1_world.jpeg)  
A world with a table with a single brick on top.

### Table with multiple bricks (table_with_4_bricks.sdf)

![drawing](img/table_4_world.jpeg)  
A world with a table with multiple bricks on top.
