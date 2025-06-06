# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import subprocess
from importlib import reload
from logging import getLogger

from PyFlow.Core import PinBase

from RCDT import camera_positions
from RCDT.Core.core import PyflowExecutor


class ChangeCamera(PyflowExecutor):
    """Node to change the camera position in the RCDT system.

    Attributes:
        name (str): The name of the node.
        input_pin (PinBase): Input pin for the camera position.
        exec_out (PinBase): Output pin to signal execution completion.
    """

    def __init__(self, name: str):
        """Initialize the ChangeCamera node with the given name.

        Args:
            name (str): The name of the node.
        """
        super().__init__(name)
        self.createInputPin("In", "ExecPin", callback=self.execute)
        self.input_pin: PinBase = self.createInputPin("position", "StringPin")
        self.exec_out: PinBase = self.createOutputPin("Out", "ExecPin")
        self.logger = getLogger("ChangeCamera")

        command_str = "ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean --timeout 2000 --req"
        self.command = command_str.split(" ")

    def execute(self, *_args: any, **_kwargs: any) -> None:
        """Execute the command to change the camera position based on the input pin data."""
        reload(camera_positions)
        position = self.input_pin.getData()
        pose = camera_positions.POSITIONS.get(position)
        if pose is None:
            self.logger.error(f"Position '{position}' does not exits.")
            return

        msg = f"pose: {{position: {{x: {pose.position.x}, y: {pose.position.y}, z: {pose.position.z}}} orientation: {{x: {pose.orientation.x}, y: {pose.orientation.y}, z: {pose.orientation.z}, w: {pose.orientation.w}}}}}"
        result = subprocess.run(self.command + [msg], check=False, capture_output=True)
        error = result.stderr.decode()
        if not error:
            self.logger.info("Changed camera position successfully.")
            self.exec_out.call()
        else:
            self.logger.error(f"Changing camera position failed: {error}")

    @staticmethod
    def category() -> str:
        """Return the category of the node."""
        return "Camera"
