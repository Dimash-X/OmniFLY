#!/usr/bin/env python3
"""
motion_capture_tracking publishes ONE array topic (/poses). The controllers need
/<name>/pose. This node splits it and checks the mocap rate (old server.yaml warning).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray


class MocapRelay(Node):
    def __init__(self):
        super().__init__('mocap_relay')
        self.declare_parameter('drone_names', ['drone1'])
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('warning_if_rate_outside', [80.0, 120.0])
        self.declare_parameter('check_period', 1.0)
        self.names = list(self.get_parameter('drone_names').value)
        self.frame = self.get_parameter('frame_id').value
        self.rng = self.get_parameter('warning_if_rate_outside').value
        self.period = self.get_parameter('check_period').value
        self.pubs = {n: self.create_publisher(PoseStamped, f'/{n}/pose', 10) for n in self.names}
        self.count = 0
        self.create_subscription(NamedPoseArray, '/poses', self.cb, qos_profile_sensor_data)
        self.create_timer(self.period, self.check)

    def cb(self, msg):
        self.count += 1
        for np_ in msg.poses:
            if np_.name in self.pubs:
                out = PoseStamped()
                out.header.stamp = msg.header.stamp
                out.header.frame_id = self.frame
                out.pose = np_.pose
                self.pubs[np_.name].publish(out)

    def check(self):
        rate = self.count / self.period
        self.count = 0
        if not (self.rng[0] <= rate <= self.rng[1]):
            self.get_logger().warn(f'Mocap rate {rate:.0f} Hz outside {self.rng}')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MocapRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
