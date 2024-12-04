# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from RCDT.Nodes.core import RosMessage, RosService, RosNode
from typing import List
import sys
import inspect
from importlib import reload
from RCDT.Nodes import messages
from RCDT.Nodes import nodes


class Point2DMsg(RosMessage):
    def __init__(self, name: str):
        reload(messages)
        messages.Point2DMsg(self)
        super().__init__(name)


class TransformMsg(RosMessage):
    def __init__(self, name: str):
        reload(messages)
        messages.TransformMsg(self)
        super().__init__(name)


class PoseStampedMsg(RosMessage):
    def __init__(self, name: str):
        reload(messages)
        messages.PoseStampedMsg(self)
        super().__init__(name)


class CollisionObjectMsg(RosMessage):
    def __init__(self, name: str):
        reload(messages)
        messages.CollisionObjectMsg(self)
        super().__init__(name)


class RvizMark(RosService):
    def __init__(self, name: str):
        reload(nodes)
        nodes.RvizMark(self)
        super().__init__(name)


class TransformPoseNode(RosService):
    def __init__(self, name: str):
        reload(nodes)
        nodes.TransformPoseNode(self)
        super().__init__(name)


class GetCameraInfo(RosNode):
    def __init__(self, name: str):
        reload(nodes)
        nodes.GetCameraInfo(self)
        super().__init__(name)


class GetImageFromTopic(RosNode):
    def __init__(self, name: str):
        reload(nodes)
        nodes.GetImageFromTopic(self)
        super().__init__(name)


class GetImageFromFile(RosNode):
    def __init__(self, name: str):
        reload(nodes)
        nodes.GetImageFromFile(self)
        super().__init__(name)


class GetImageFromList(RosNode):
    def __init__(self, name: str):
        reload(nodes)
        nodes.GetImageFromList(self)
        super().__init__(name)


class PublishImage(RosNode):
    def __init__(self, name: str):
        reload(nodes)
        nodes.PublishImage(self)
        super().__init__(name)


class NormalizeImageNode(RosService):
    def __init__(self, name: str):
        reload(nodes)
        nodes.NormalizeImageNode(self)
        super().__init__(name)


class Segment(RosService):
    def __init__(self, name: str):
        reload(nodes)
        nodes.Segment(self)
        super().__init__(name)


class Filter(RosService):
    def __init__(self, name: str):
        reload(nodes)
        nodes.Filter(self)
        super().__init__(name)


class DefineCentroidNode(RosService):
    def __init__(self, name: str):
        reload(nodes)
        nodes.DefineCentroidNode(self)
        super().__init__(name)


class PoseFromPixelNode(RosService):
    def __init__(self, name: str):
        reload(nodes)
        nodes.PoseFromPixelNode(self)
        super().__init__(name)


class MoveitMoveRobot(RosService):
    def __init__(self, name: str):
        reload(nodes)
        nodes.MoveitMoveRobot(self)
        super().__init__(name)


class MoveitAddObject(RosService):
    def __init__(self, name: str):
        reload(nodes)
        nodes.MoveitAddObject(self)
        super().__init__(name)


class MoveitClearObjects(RosService):
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
