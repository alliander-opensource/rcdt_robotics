import contextlib
from threading import Event, Thread
from typing import Generator

import docker
import python_on_whales as whales


class Streamer:
    def __init__(self, stream: Generator[bytes, None, None]):
        self.stream = stream
        self.active = True
        self.thread = Thread(target=self.stream_output)
        self.thread.start()

    def stream_output(self):
        for data in self.stream:
            if not self.active:
                break
            print(data.decode("utf-8"))

    def stop(self):
        self.active = False


class LaunchSystem:
    def __init__(self):
        whales.docker.compose.up(detach=True)
        self.client = docker.from_env()
        self.streamers: list[Streamer] = []

    def launch_gazebo(self):
        platforms = ["panther"]
        positions = ["0,0,0.2"]
        orientations = ["0,0,0"]
        parents = ["none"]
        parent_links = ["none"]

        args = [
            f"platforms:={' '.join(platforms)}",
            f"positions:={' '.join(positions)}",
            f"orientations:={' '.join(orientations)}",
            f"parents:={' '.join(parents)}",
            f"parent_links:={' '.join(parent_links)}",
            "load_gazebo_ui:=True",
        ]
        cmd = f"ros2 launch rcdt_gazebo gazebo_robot.launch.py {' '.join(args)}"

        rcdt_gazebo = self.client.containers.get("rcdt_gazebo")
        _, stream = rcdt_gazebo.exec_run(f"bash -it -c '{cmd}'", stream=True)
        self.streamers.append(Streamer(stream))

    def launch_husarion(self):
        cmd = "ros2 launch rcdt_husarion husarion.launch.py"

        rcdt_husarion = self.client.containers.get("rcdt_husarion")
        _, stream = rcdt_husarion.exec_run(f"bash -it -c '{cmd}'", stream=True)
        self.streamers.append(Streamer(stream))

    def stop_on_keyboard_interrupt(self):
        with contextlib.suppress(KeyboardInterrupt):
            Event().wait()
        for streamer in self.streamers:
            streamer.stop()
        whales.docker.compose.down(timeout=1)


launch_system = LaunchSystem()
launch_system.launch_gazebo()
launch_system.launch_husarion()
launch_system.stop_on_keyboard_interrupt()
