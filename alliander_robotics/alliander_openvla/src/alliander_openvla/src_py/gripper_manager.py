#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
# This class handles the gripper's movements

from rclpy.action import ActionClient
from alliander_interfaces.action import TriggerAction


class GripperManager:

    def __init__(self, node, open_client: ActionClient, close_client: ActionClient):
        self.node = node
        self.open_client = open_client
        self.close_client = close_client

        self.current_state = None
        self.busy = False
        self.timer = None

        # thresholds (can be tweaked)
        self.open_thresh = 0.2
        self.close_thresh = -0.2


    def handle(self, gripper_value):
        if gripper_value is None:
            return # Do not move the gripper if it has no value

        if self.busy:
            return # Do not move the gripper again when still moving

        desired = self.interpret(gripper_value)

        if desired is None:
            return

        if desired == self.current_state:
            return # Do not send new request if already in that state

        self.current_state = desired

        if desired == "open":
            client = self.open_client
            reset_time = 0.5
        else:
            client = self.close_client
            reset_time = 1.0

        if not client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn(f"{desired} action server not available")
            return

        goal = TriggerAction.Goal()

        self.node.get_logger().info(f"Gripper: {desired}")

        self.busy = True
        future = client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

        # Reset busy flag after time
        self.timer = self.node.create_timer(
            reset_time,
            self.reset_busy_once
        )

    
    def interpret(self, value):
        """Interpret the action value, where anything below the lower bound is set to close and vice versa."""
        if value > self.open_thresh:
            return "open"
        elif value < self.close_thresh:
            return "close"
        return None
    
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn("Gripper goal rejected")
            self.busy = False
            return

        self.node.get_logger().info("Gripper goal accepted")

    def reset_busy_once(self):
        self.busy = False
        self.node.get_logger().info("Gripper ready again")

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None


