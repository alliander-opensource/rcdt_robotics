from rcdt_actions.definitions import Action
from std_srvs.srv import Trigger

open_gripper = Action("/open_gripper", Trigger)
close_gripper = Action("/close_gripper", Trigger)
