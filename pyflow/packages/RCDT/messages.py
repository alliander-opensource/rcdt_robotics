import geometry_msgs.msg as geometry


messages: set[type] = set()


def add(message: type) -> None:
    messages.add(message)


add(geometry.Transform)
add(geometry.PoseStamped)
