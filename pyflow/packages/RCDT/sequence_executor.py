from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from RCDT.Core.core import PyflowRosBridge, PyflowNonBlockingExecutor
from PyFlow.Core import PinBase
from logging import getLogger
from rcdt_messages.action import Sequence
import time

logger = getLogger(__name__)

TIMEOUT_SERVER = 3
TIMEOUT_ACTION = 10


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
        if not self.action_client.wait_for_server(TIMEOUT_SERVER):
            logger.error(f"Action server '{self.server_name}' is not available.")
            return

        future = self.action_client.send_goal_async(goal, self.feedback_callback)
        if not self.sequence_finished(future):
            return

        goal_handle: ClientGoalHandle = future.result()
        response: Sequence.Impl.GetResultService.Response = goal_handle.get_result()
        result = response.result
        if result.success:
            logger.info(result.message)
            self.exec_out.call()
        else:
            logger.error(result.message)

    def sequence_finished(self, future: Future) -> bool:
        self.last_response = time.time()
        while not future.done():
            if time.time() - self.last_response > TIMEOUT_ACTION:
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
