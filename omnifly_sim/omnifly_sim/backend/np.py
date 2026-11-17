from __future__ import annotations

import math

import numpy as np
from rclpy.node import Node

from ..sim_data_types import Action, State


class Backend:
    """
    Simple physics backend emulating Betaflight ANGLE mode, in numpy.

    roll/pitch stick -> target tilt angle (first-order lag), yaw stick -> yaw rate,
    throttle -> collective thrust (linear). Translation: point mass with linear drag.
    """

    def __init__(self, node: Node, names: list[str], states: list[State], params: dict = None):
        self.node = node
        self.names = names
        self.t = 0.0
        self.dt = 0.002
        params = params or {}
        self.uavs = [Quadrotor(state, params) for state in states]

    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        self.t += self.dt
        next_states = []
        for uav, action in zip(self.uavs, actions):
            uav.step(action, self.dt)
            next_states.append(uav.state)
        return next_states

    def shutdown(self):
        pass


def euler_to_quat(roll, pitch, yaw):
    """Return [w, x, y, z] for ZYX (yaw-pitch-roll) Euler angles."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])


class Quadrotor:
    """Point-mass quadrotor with an Betaflight-like angle-mode inner loop."""

    def __init__(self, state: State, params: dict):
        # APPROXIMATE values for the 2.5" OmniFLY drone: measure and update in sim.yaml
        self.mass = params.get('mass', 0.08)                      # kg
        twr = params.get('thrust_to_weight', 3.0)
        self.g = 9.81
        self.max_thrust = twr * self.mass * self.g                # N at throttle = 1
        self.max_angle = math.radians(params.get('max_angle_deg', 55.0))
        self.max_yaw_rate = math.radians(params.get('max_yaw_rate_deg', 200.0))
        self.tau = params.get('attitude_tau', 0.06)               # s
        self.drag = params.get('drag', 0.1)                       # 1/s
        self.state = state
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def step(self, action: Action, dt: float):
        s = self.state
        if action.armed:
            r_cmd, p_cmd, thr, y_cmd = np.clip(action.cmd, [-1, -1, 0, -1], [1, 1, 1, 1])
        else:
            r_cmd = p_cmd = y_cmd = thr = 0.0

        k = min(dt / self.tau, 1.0)
        r_prev, p_prev = self.roll, self.pitch
        self.roll += (r_cmd * self.max_angle - self.roll) * k
        self.pitch += (p_cmd * self.max_angle - self.pitch) * k
        # Betaflight: yaw stick right = clockwise = negative yaw in ENU/FLU
        yaw_rate = -y_cmd * self.max_yaw_rate
        self.yaw += yaw_rate * dt

        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        z_body = np.array([cr * sp * cy + sr * sy,
                           cr * sp * sy - sr * cy,
                           cr * cp])

        thrust = thr * self.max_thrust
        acc = z_body * thrust / self.mass - np.array([0, 0, self.g]) - self.drag * s.vel
        vel_next = s.vel + acc * dt
        pos_next = s.pos + vel_next * dt

        if pos_next[2] <= 0.0 and vel_next[2] <= 0.0:       # on the ground
            pos_next[2] = 0.0
            vel_next = np.zeros(3)
            self.roll = self.pitch = 0.0
            r_prev = p_prev = 0.0

        s.pos = pos_next
        s.vel = vel_next
        s.quat = euler_to_quat(self.roll, self.pitch, self.yaw)
        s.omega = np.array([(self.roll - r_prev) / dt, (self.pitch - p_prev) / dt, yaw_rate])
