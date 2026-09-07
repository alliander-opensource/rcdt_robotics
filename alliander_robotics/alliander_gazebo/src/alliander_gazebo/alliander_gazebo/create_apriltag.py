# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import PIL
import requests
import trimesh
import trimesh.visual
from alliander_utilities.config_objects import Apriltag
from PIL import Image
from rclpy.node import Node


def create_apriltag(tag: Apriltag, node: Node) -> None:
    """Create a 3D model of an AprilTag using trimesh and export it as a GLB file.

    Args:
        tag (Apriltag): The AprilTag configuration object.
        node (Node): The ROS2 node used for logging.
    """
    filename = tag.family.replace("h", "_") + f"_{tag.id:05d}" + ".png"
    url = f"https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag{tag.family}/tag{filename}"
    try:
        img = np.array(Image.open(requests.get(url, stream=True).raw))
    except PIL.UnidentifiedImageError as _:
        node.get_logger().error(
            "Failed to download AprilTag image. Falling back to red plane."
        )
        img = np.array([[[255, 0, 0]]])
    rows, cols, _ = img.shape
    size = tag.size / (rows - 2)

    meshes = []
    transform = np.eye(4)
    transform[0, -1] = -(rows * size) / 2 + size / 2
    transform[1, -1] = -(cols * size) / 2 + size / 2
    for row in range(rows):
        for col in range(cols):
            mesh: trimesh.Trimesh = trimesh.creation.box(
                [size, size, tag.thickness], transform
            )
            color = img[row, col]
            material = trimesh.visual.material.PBRMaterial(baseColorFactor=color)
            mesh.visual = trimesh.visual.TextureVisuals(material=material)
            meshes.append(mesh)
            transform[1, -1] += size
        transform[1, -1] = -(cols * size) / 2 + size / 2
        transform[0, -1] += size

    combined_mesh = trimesh.util.concatenate(meshes)
    combined_mesh.export("/tmp/apriltag.glb", file_type="glb")
