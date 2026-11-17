from __future__ import annotations

import time

import numpy as np

from .sim_data_types import Action, State


def yaw_to_quat(yaw):
    """Return [w, x, y, z] for a pure yaw rotation."""
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


class SimDrone:
    """
    Per-drone container replacing CrazyflieSIL.

    Holds the latest RC command, arm flag and goal received over ROS topics, i.e. exactly
    what the real crsf_bridge would receive.
    """

    def __init__(self, name: str, initial_position, cmd_timeout: float = 0.3):
        self.name = name
        self.cmd_timeout = cmd_timeout
        self.state = State(np.array(initial_position, dtype=float))
        self.cmd = np.zeros(4)          # roll, pitch, throttle, yaw
        self.armed = False
        self.last_cmd = 0.0
        self.goal = None                # (pos[3], yaw)

    # --- ROS callbacks (called by the server) ---
    def on_cmd(self, msg):
        self.cmd = np.array([msg.linear.y, msg.linear.x, msg.linear.z, msg.angular.z])
        self.last_cmd = time.monotonic()

    def on_arm(self, msg):
        self.armed = bool(msg.data)

    def on_goal(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = 2.0 * np.arctan2(q.z, q.w)
        self.goal = (np.array([p.x, p.y, p.z]), yaw)

    def emergency(self):
        self.armed = False
        self.cmd = np.zeros(4)

    # --- simulator interface ---
    def getSetpoint(self) -> State:
        if self.goal is None:
            return self.state
        return State(self.goal[0], np.zeros(3), yaw_to_quat(self.goal[1]), np.zeros(3))

    def executeController(self) -> Action:
        alive = (time.monotonic() - self.last_cmd) < self.cmd_timeout   # same failsafe as bridge
        if not (alive and self.armed):
            return Action(np.zeros(4), armed=False)
        return Action(self.cmd, armed=True)

    def setState(self, state: State):
        self.state = state
