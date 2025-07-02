#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time 

EE_FRAME = "fr3_hand"
PUBLISH_PERIOD = 20.0   # seconds between publishes

API_URL = "http://localhost:8000/predict"
PROMPT  = "In: What action should the robot take to pick up the grey brick from the yellow table?\nOut:"

class VLAServo(Node):
    def __init__(self) -> None:
        super().__init__("vla_servo_http")
        self.bridge = CvBridge()
        self.latest_image = None

        # Subscribe to your camera topic
        self.create_subscription(
            Image, "/world/empty/model/overhead_camera/link/camera_link/sensor/camera_sensor/image", self._on_image, 1
        )

        # Publishers
        self.pub_twist   = self.create_publisher(TwistStamped,
                                                 "/franka/servo_node/delta_twist_cmds", 10)
        self.pub_gripper = self.create_publisher(Float64,
                                                 "/franka/franka_gripper_controller/command", 10)

        # Timer
        self.create_timer(PUBLISH_PERIOD, self.step)
        
        self._last_step_time = time.time()

    def _on_image(self, msg: Image):
        # store the latest camera frame
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def step(self) -> None:
        now = time.time()
        interval = now - self._last_step_time
        self._last_step_time = now

        # Print out how long since the last invocation:
        print(f"[VLAServo] step() called!  Δt = {interval:.3f} s")
        if self.latest_image is None:
            return

        # 1) encode latest_image as JPEG in memory
        is_success, buffer = cv2.imencode(".jpg", self.latest_image)
        if not is_success:
            self.get_logger().error("Failed to encode image")
            return
        jpg_bytes = buffer.tobytes()

        # 2) call FastAPI predict endpoint
        files = {"image_file": ("frame.jpg", jpg_bytes, "image/jpeg")}
        data  = {"prompt": PROMPT}
        try:
            start = time.time()
            resp = requests.post(API_URL, files=files, data=data, timeout=20)
            resp.raise_for_status()
            action = resp.json()["action"]
            elapsed = time.time() - start
            print(f"HTTP request took {elapsed:.3f} seconds")
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            return

        # 3) unpack the 7 deltas
        dx, dy, dz, droll, dpitch, dyaw, dgrip = action
        # dx, dy, dz, droll, dpitch, dyaw, dgrip = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Placeholder values for testing

        # 4) publish TwistStamped
        twist = TwistStamped()
        twist.header.stamp    = self.get_clock().now().to_msg()
        twist.header.frame_id = EE_FRAME
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = dx, dy, dz
        twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = (
            droll, dpitch, dyaw
        )
        self.pub_twist.publish(twist)
        print(f"Published twist: {twist}")
        # # 5) (optional) gripper command
        # jaw = max(0.0, min(0.08, (dgrip + 1) * 0.04))
        # self.pub_gripper.publish(Float64(data=jaw))


def main() -> None:
    rclpy.init()
    node = VLAServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
