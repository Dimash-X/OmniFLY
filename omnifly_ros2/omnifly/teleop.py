#!/usr/bin/env python3
"""
Joystick teleop (replaces crazyswarm2 teleop, cmd_vel_world mode).
Moves the swarm centre; publishes /<name>/goal = centre + formation offset.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from std_srvs.srv import Trigger


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        d = self.declare_parameter
        d('drone_names', ['drone1'])
        d('offsets', [0.0, 0.0, 0.0])
        d('frequency', 100)
        for k, ax in (('x', 5), ('y', 4), ('z', 2), ('yaw', 1)):
            d(f'cmd_vel_world.{k}_velocity.axis', ax)
            d(f'cmd_vel_world.{k}_velocity.max', 0.5)
            d(f'cmd_vel_world.{k}_velocity.deadband', 0.1)
        d('cmd_vel_world.x_limit', [-1.0, 1.0])
        d('cmd_vel_world.y_limit', [-1.0, 1.0])
        d('cmd_vel_world.z_limit', [0.1, 2.0])
        d('initial_position', [0.0, 0.0, 0.5])
        d('takeoff_button', 7); d('land_button', 6); d('emergency_button', 1)

        g = lambda k: self.get_parameter(k).value
        self.names = list(g('drone_names'))
        o = g('offsets')
        self.offs = {n: o[3 * i:3 * i + 3] for i, n in enumerate(self.names)}
        self.cfg = {k: (g(f'cmd_vel_world.{k}_velocity.axis'),
                        g(f'cmd_vel_world.{k}_velocity.max'),
                        g(f'cmd_vel_world.{k}_velocity.deadband')) for k in ('x', 'y', 'z', 'yaw')}
        self.lim = [g('cmd_vel_world.x_limit'), g('cmd_vel_world.y_limit'),
                    g('cmd_vel_world.z_limit')]
        self.c = list(g('initial_position'))
        self.yaw = 0.0
        self.btn = {'takeoff': g('takeoff_button'), 'land': g('land_button'),
                    'emergency': g('emergency_button')}
        self.prev, self.joy = {}, None
        self.dt = 1.0 / g('frequency')

        self.pubs = {n: self.create_publisher(PoseStamped, f'/{n}/goal', 10) for n in self.names}
        self.emerg = self.create_publisher(Empty, '/omnifly/emergency', 10)
        self.takeoff_cl = {n: self.create_client(Trigger, f'/{n}/takeoff') for n in self.names}
        self.land_cl = {n: self.create_client(Trigger, f'/{n}/land') for n in self.names}
        self.create_subscription(Joy, '/joy', lambda m: setattr(self, 'joy', m), 10)
        self.create_timer(self.dt, self.tick)

    def axis(self, key):
        ax, mx, db = self.cfg[key]
        if self.joy is None or ax >= len(self.joy.axes):
            return 0.0
        v = self.joy.axes[ax] * mx
        return 0.0 if abs(v) < db else v

    def pressed(self, name):
        b = self.btn[name]
        now = bool(self.joy and b < len(self.joy.buttons) and self.joy.buttons[b])
        edge = now and not self.prev.get(name, False)
        self.prev[name] = now
        return edge

    def tick(self):
        if self.joy is None:
            return
        if self.pressed('emergency'):
            self.emerg.publish(Empty())
        if self.pressed('takeoff'):
            for cl in self.takeoff_cl.values():
                cl.call_async(Trigger.Request())
        if self.pressed('land'):
            for cl in self.land_cl.values():
                cl.call_async(Trigger.Request())
        self.c[0] = clamp(self.c[0] + self.axis('x') * self.dt, *self.lim[0])
        self.c[1] = clamp(self.c[1] + self.axis('y') * self.dt, *self.lim[1])
        self.c[2] = clamp(self.c[2] + self.axis('z') * self.dt, *self.lim[2])
        self.yaw += self.axis('yaw') * self.dt
        for n in self.names:
            g = PoseStamped()
            g.header.stamp = self.get_clock().now().to_msg()
            o = self.offs[n]
            g.pose.position.x = self.c[0] + o[0]
            g.pose.position.y = self.c[1] + o[1]
            g.pose.position.z = self.c[2] + o[2]
            g.pose.orientation.z = math.sin(self.yaw / 2)
            g.pose.orientation.w = math.cos(self.yaw / 2)
            self.pubs[n].publish(g)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Teleop())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
