#!/usr/bin/env python3
"""Replaces follower_cf.py: follower goal = leader pose + fixed offset."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FormationFollower(Node):
    def __init__(self):
        super().__init__('formation_follower')
        self.declare_parameter('drone_name', 'drone2')
        self.declare_parameter('leader_name', 'drone1')
        self.declare_parameter('offset', [0.4, 0.0, 0.0])   # x, y, z in world frame
        n = self.get_parameter('drone_name').value
        l = self.get_parameter('leader_name').value
        self.off = self.get_parameter('offset').value
        self.pub = self.create_publisher(PoseStamped, f'/{n}/goal', 10)
        self.create_subscription(PoseStamped, f'/{l}/pose_rel', self.cb, 10)

    def cb(self, msg):
        g = PoseStamped()
        g.header = msg.header
        g.pose.position.x = msg.pose.position.x + self.off[0]
        g.pose.position.y = msg.pose.position.y + self.off[1]
        g.pose.position.z = msg.pose.position.z + self.off[2]
        g.pose.orientation = msg.pose.orientation
        self.pub.publish(g)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FormationFollower())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
