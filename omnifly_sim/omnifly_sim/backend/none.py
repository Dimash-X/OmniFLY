from __future__ import annotations

from rclpy.node import Node

from ..sim_data_types import Action, State


class Backend:
    """Drones jump exactly to their /<name>/goal (no physics). Fast, for image generation."""

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.t = 0
        self.dt = 0.1

    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        # advance the time
        self.t += self.dt

        # pretend we were able to follow desired states perfectly
        return states_desired

    def shutdown(self):
        pass
