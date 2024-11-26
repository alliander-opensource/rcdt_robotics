from RCDT.Nodes.core import RosNode, RosMessage
from typing import List
import sys
import inspect
from importlib import reload
from RCDT.Nodes import messages
from RCDT.Nodes import nodes


class PoseStampedMsg(RosMessage):
    def __init__(self, name: str):
        super().__init__(name)
        reload(messages)
        messages.PoseStampedMsg(self)


class CollisionObjectMsg(RosMessage):
    def __init__(self, name: str):
        super().__init__(name)
        reload(messages)
        messages.CollisionObjectMsg(self)


class RvizMark(RosNode):
    def __init__(self, name: str):
        reload(nodes)
        nodes.RvizMark(self)
        super().__init__(name)


class GetImage(RosNode):
    def __init__(self, name: str):
        reload(nodes)
        nodes.GetImage(self)
        super().__init__(name)


class Segment(RosNode):
    def __init__(self, name: str):
        reload(nodes)
        nodes.Segment(self)
        super().__init__(name)


class MoveitMove(RosNode):
    def __init__(self, name: str):
        reload(nodes)
        nodes.MoveitMove(self)
        super().__init__(name)


class MoveitAddObject(RosNode):
    def __init__(self, name: str):
        reload(nodes)
        nodes.MoveitAddObject(self)
        super().__init__(name)


class MoveitClearObjects(RosNode):
    def __init__(self, name: str):
        reload(nodes)
        nodes.MoveitClearObjects(self)
        super().__init__(name)


def get_nodes() -> List[object]:
    module = sys.modules[__name__]
    module_name = module.__name__
    nodes = {}
    for _name, obj in inspect.getmembers(module):
        if inspect.isclass(obj) and obj.__module__ == module_name:
            nodes[obj.__name__] = obj
    return nodes
