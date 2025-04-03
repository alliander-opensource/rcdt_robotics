# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import time
from logging import getLogger

from PyFlow.Core import PinBase
from rcdt_messages.action import Sequence
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future

from RCDT.Core.core import PyflowNonBlockingExecutor, PyflowRosBridge

logger = getLogger(__name__)

SERVICE_AVAILABLE_TIMEOUT = 3
SERVICE_RESPONSE_TIMEOUT = 20


class SequenceExecutor(PyflowNonBlockingExecutor):
    def __init__(self, name: str):
        super().__init__(name)
        self.server_name = "/action_executor"

        self.createInputPin("In", "ExecPin", callback=self.execute_parallel)
        self.exec_out: PinBase = self.createOutputPin("Out", "ExecPin")
        self.input_pin: PinBase = self.createInputPin("sequence", "StringPin")

        self.action_client = ActionClient(
            PyflowRosBridge.node, Sequence, self.server_name
        )

    def execute(self, *_args: any, **_kwargs: any) -> None:
        goal = Sequence.Goal()
        goal.sequence = self.input_pin.getData()

        logger.info(f"Starting execution of sequence '{goal.sequence}'.")
        if not self.action_client.wait_for_server(SERVICE_AVAILABLE_TIMEOUT):
            logger.error(f"Action server '{self.server_name}' is not available.")
            return

        future_goal_handle = self.action_client.send_goal_async(
            goal, self.feedback_callback
        )
        if not self.future_done(future_goal_handle):
            return
        goal_handle: ClientGoalHandle = future_goal_handle.result()

        future_service_response: Future = goal_handle.get_result_async()
        if not self.future_done(future_service_response):
            return
        service_response: Sequence.Impl.GetResultService.Response = (
            future_service_response.result()
        )

        result = service_response.result
        if result.success:
            logger.info(result.message)
            self.exec_out.call()
        else:
            logger.error(result.message)

    def future_done(self, future: Future) -> bool:
        self.last_response = time.time()
        while not future.done():
            if time.time() - self.last_response > SERVICE_RESPONSE_TIMEOUT:
                logger.error("Timeout on execution of sequence.")
                return False
            time.sleep(1)
        return True

    def feedback_callback(self, feedback_msg: Sequence.Impl.FeedbackMessage) -> None:
        feedback = feedback_msg.feedback
        if feedback.success:
            logger.info(feedback.message)
        else:
            logger.error(feedback.message)
        self.last_response = time.time()

    @staticmethod
    def category() -> str:
        return "Custom"
