<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# RCDT Detection

This repository can be used for detecting objects using a camera, currently a RealSense.

## Usage

This repository contains multiple services:
- /define_centroid
- /filter_masks
- /get_bounding_box_2d
- /get_largest_contour
- /get_mean_hue
- /get_rectangle_factor
- /pose_from_pixel
- /segment_image
- /split_rgbd

You can run these services by running the following in a sourced terminal:

```bash
ros2 run rcdt_detection <node_name>_node.py
```

## License

This project is licensed under the Apache License Version 2.0 - see [LICENSE](LICENSE) for details.

## Contributing

Please read CODE_OF_CONDUCT, CONTRIBUTING, and PROJECT GOVERNANCE located in the overarching [RCDT robotics](https://github.com/alliander-opensource/rcdt_robotics) project repository for details on the process for submitting pull requests to us. 