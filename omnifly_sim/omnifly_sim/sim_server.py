#!/usr/bin/env python3
"""
OmniFLY simulation server (replaces crazyflie_server for simulation).

Stands in for crsf_bridge + the motion capture system:
  in : /<name>/rc_cmd, /<name>/arm, /<name>/goal, /omnifly/emergency
  out: /<name>/pose  (what mocap_relay would publish)
The real position_controller runs unchanged on top of it.
"""
import importlib
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
import yaml

from .sim_drone import SimDrone


class SimServer(Node):

    def __init__(self):
        super().__init__('omnifly_sim')
        default_cfg = os.path.join(get_package_share_directory('omnifly_sim'),
                                   'config', 'sim.yaml')
        self.declare_parameter('sim_config', default_cfg)
        with open(self.get_parameter('sim_config').value) as f:
            cfg = yaml.safe_load(f)
        sim = cfg['sim']
        drones_yaml = os.path.join(get_package_share_directory('omnifly'),
                                   'config', 'drones.yaml')
        with open(drones_yaml) as f:
            robots = yaml.safe_load(f)['robots']
        robots = {n: r for n, r in robots.items() if r.get('enabled')}
        names = list(robots)

        self.drones = {n: SimDrone(n, robots[n]['initial_position'],
                                   sim.get('cmd_timeout', 0.3)) for n in names}
        initial_states = [self.drones[n].state for n in names]

        # backend and visualizations are loaded dynamically (as in crazyflie_sim)
        module = importlib.import_module('.backend.' + sim['backend'], package='omnifly_sim')
        self.backend = module.Backend(self, names, initial_states,
                                      cfg.get('quadrotor', {})) \
            if sim['backend'] == 'np' else module.Backend(self, names, initial_states)

        self.visualizations = []
        for key, vparams in sim.get('visualizations', {}).items():
            if vparams.get('enabled'):
                vmod = importlib.import_module('.visualization.' + key, package='omnifly_sim')
                self.visualizations.append(
                    vmod.Visualization(self, vparams, names, initial_states))

        self.noise = sim.get('pose_noise_std', 0.0)
        self.pose_pubs = {}
        for n, d in self.drones.items():
            self.create_subscription(Twist, f'/{n}/rc_cmd', d.on_cmd, 10)
            self.create_subscription(Bool, f'/{n}/arm', d.on_arm, 10)
            self.create_subscription(PoseStamped, f'/{n}/goal', d.on_goal, 10)
            self.pose_pubs[n] = self.create_publisher(PoseStamped, f'/{n}/pose', 10)
        self.create_subscription(Empty, '/omnifly/emergency', self._emergency, 10)

        self.last_wall = time.monotonic()
        self.is_shutdown = False
        self.create_timer(1.0 / sim.get('pose_rate_hz', 100.0), self._timer_callback)
        self.get_logger().info(f'Simulating {names} with backend {sim["backend"]}')

    def _emergency(self, _):
        for d in self.drones.values():
            d.emergency()

    def _timer_callback(self):
        # real-time: advance the physics by the wall time that has passed
        now = time.monotonic()
        elapsed = min(now - self.last_wall, 0.1)
        self.last_wall = now
        n = max(1, int(round(elapsed / self.backend.dt)))
        for _ in range(n):
            desired = [d.getSetpoint() for d in self.drones.values()]
            actions = [d.executeController() for d in self.drones.values()]
            states = self.backend.step(desired, actions)
            for state, d in zip(states, self.drones.values()):
                d.setState(state)

        stamp = self.get_clock().now().to_msg()
        for (name, d), pub in zip(self.drones.items(), self.pose_pubs.values()):
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'world'
            p = d.state.pos + np.random.normal(0.0, self.noise, 3) if self.noise else d.state.pos
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
            q = d.state.quat
            msg.pose.orientation.w, msg.pose.orientation.x = float(q[0]), float(q[1])
            msg.pose.orientation.y, msg.pose.orientation.z = float(q[2]), float(q[3])
            pub.publish(msg)

        for vis in self.visualizations:
            vis.step(self.backend.time(), states, desired, actions)

    def on_shutdown_callback(self):
        if not self.is_shutdown:
            self.backend.shutdown()
            for vis in self.visualizations:
                vis.shutdown()
            self.is_shutdown = True


def main(args=None):
    rclpy.init(args=args)
    server = SimServer()
    rclpy.get_default_context().on_shutdown(server.on_shutdown_callback)
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.on_shutdown_callback()
    finally:
        rclpy.try_shutdown()
        server.destroy_node()


if __name__ == '__main__':
    main()
