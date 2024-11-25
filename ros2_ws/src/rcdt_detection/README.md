<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# RCDT Detection

This repository can be used for detecting objects with an RGBD-camera, currently a RealSense-camera. It is intended to be used alongside [other RCDT robotics projects](<Link to RCDT robotics repo>).

It contains two ROS packages: one for the actual detection, and one containing required custom message- and service definitions.

# Usage

This repository contains two services: `/detect_objects` and `/segment_image`.

You can launch this service, and the required RealSense-node, using:
```bash
ros2 launch rcdt_detection detect.launch.py
```

Once the service is running, you can call it using:
```bash
ros2 service call /detect_objects rcdt_detection_msgs/srv/DetectObjects {}
```

# License

This project is licensed under the Apache License Version 2.0 - see [LICENSE](LICENSE) for details.

## Contributing

Please read CODE_OF_CONDUCT, CONTRIBUTING, and PROJECT GOVERNANCE located in the overarching [RCDT robotics](https://github.com/alliander-opensource/rcdt_robotics) project repository for details on the process for submitting pull requests to us. 