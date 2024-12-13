<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# RCDT Utilities

This repository contains ROS2-based utility functions and nodes that can be used for ROS2 software development for any robot. This avoids duplication of often repeated code between different robotic specific repositories.

## Usage

This repository contains multiple services:
- /manipulate_pose/express_pose_in_other_frame
- /manipulate_pose/transform_pose
- /manipulate_pose/transform_pose_relative
- /moveit_controller/move_hand_to_pose
- /moveit_controller/move_to_configuration
- /moveit_controller/add_object
- /moveit_controller/clear_objects
- /is_between

You can run these services by running the following in a sourced terminal:

```bash
ros2 run rcdt_detection <node_name>_node.py
```

## Moveit Controller Node

This node can be used to use code-wise Moveit control. For Franka:

`ros2 launch rcdt_franka franka.launch.py moveit:=node`

By selecting option *node* for the moveit flag, the moveit_controller node will be started. One can move the robot using an Action call, for example:

`ros2 action send_goal --feedback /moveit_controller rcdt_utilities_msgs/action/Moveit "{goal_pose: {header: {frame_id: fr3_link0}, pose: {position: {x: 0.28, y: 0.2, z: 0.5}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}}"`

## License

This project is licensed under the Apache License Version 2.0 - see [LICENSE](LICENSE) for details.

## Contributing

Please read CODE_OF_CONDUCT, CONTRIBUTING, and PROJECT GOVERNANCE located in the overarching [RCDT robotics](https://github.com/alliander-opensource/rcdt_robotics) project repository for details on the process for submitting pull requests to us.
