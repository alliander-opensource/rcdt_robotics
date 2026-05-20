#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np
from PIL import Image as PILImage
from sensor_msgs.msg import Image


def image_to_pil(msg):
    image = PILImage.frombuffer(
    "RGB",
    (msg.width, msg.height),
    msg.data,
    "raw",
    "RGB",
    0,
    1,
    )
    return image


def cv_to_ros_image(cv_image, frame_id="camera"):
    msg = Image()
    
    msg.height = cv_image.shape[0]
    msg.width = cv_image.shape[1]
    msg.encoding = "bgr8"   # OpenCV default
    msg.is_bigendian = False
    msg.step = cv_image.shape[1] * 3  # width * channels
    msg.data = cv_image.tobytes()
    
    return msg

def convert_posedelta_to_velo(action: np.ndarray, dt_model: float=0.2):
        """Function to convert EE pose delta's (OpenVLA's output) to velocities
        which can be published to MoveIt servo node as TwistStamped commands.
        
        Args:
            action (array): The action array outputted by OpenVLA; containing EE pose delta's
            dt_model (float): Timestep used during model training. Defaults to 0.2s (5Hz)
        """
        dx, dy, dz = action[:3]
        droll, dpitch, dyaw = action[3:6]

        # Velocity = position change (action delta's) / time (dt_model)
        vx = dx / dt_model
        vy = dy / dt_model
        vz = dz / dt_model

        vroll = droll / dt_model
        vpitch = dpitch / dt_model
        vyaw = dyaw / dt_model
        
        # Gripper
        gripper = action[6] if len(action) > 6 else None

        # Then, we must convert EE frame to base frame. NOTE: Not needed because of apply_twist_commands_about_ee_frame: true in moveit servo config???
        # try:
        #     transform = self.tf_buffer.lookup_transform(
        #             "franka/world",     # base frame
        #             "franka/fr3_hand_tcp",
        #             rclpy.time.Time()
        #         )

        #     q = transform.transform.rotation
        #     rot = Rotation.from_quat([q.x, q.y, q.z, q.w])

        #     # Step 3 — convert linear velocity to base frame
        #     v_base = rot.apply([vx, vy, vz])
        
        # except Exception as e:
        #         self.get_logger().warn(f"transform frames failed: {e}")
        #         v_base = [vx, vy, vz]  # fallback wiithout transform

        v_base = (vx, vy, vz)
        w_base = (vroll, vpitch, vyaw)

        # Could also rotate the angular velocities. However, this is less important. For simplicity, this is now commented out
        # w_base = rot.apply([vroll, vpitch, vyaw])
        # return v_base, w_base, gripper
        return v_base, w_base, gripper

