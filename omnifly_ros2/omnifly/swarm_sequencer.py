#!/usr/bin/env python3
"""
Replaces begin_operation.py / trajectory.py / trajectory_publisher.py.
Moves the whole swarm through waypoints, each drone holding its own formation offset
(= its own "ommatidium" position). Next waypoint is sent once ALL drones are in range.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class SwarmSequencer(Node):
    def __init__(self):
        super().__init__('swarm_sequencer')
        self.declare_parameter('drone_names', ['drone1', 'drone2', 'drone3'])
        # flat lists: x,y,z per waypoint / per drone
        self.declare_parameter('waypoints', [0.0, 0.0, 0.5,  0.3, 0.0, 0.5,
                                             0.3, 0.3, 0.6,  0.0, 0.3, 0.5])
        self.declare_parameter('offsets', [0.0, 0.0, 0.0,  0.4, 0.0, 0.0,  0.0, 0.4, 0.0])
        self.declare_parameter('threshold', 0.08)
        self.declare_parameter('hold_time', 2.0)
        self.declare_parameter('loop', False)

        self.names = self.get_parameter('drone_names').value
        w = self.get_parameter('waypoints').value
        o = self.get_parameter('offsets').value
        self.wps = [w[i:i + 3] for i in range(0, len(w), 3)]
        self.offs = {n: o[3 * i:3 * i + 3] for i, n in enumerate(self.names)}
        self.th = self.get_parameter('threshold').value
        self.hold = self.get_parameter('hold_time').value
        self.loop = self.get_parameter('loop').value
        self.idx, self.reached_t = 0, None
        self.pos = {}
        self.pubs = {n: self.create_publisher(PoseStamped, f'/{n}/goal', 10) for n in self.names}
        for n in self.names:
            self.create_subscription(PoseStamped, f'/{n}/pose_rel',
                                     lambda m, n=n: self.pos.__setitem__(n, m.pose.position), 10)
        self.create_timer(0.1, self.tick)

    def target(self, n):
        w, o = self.wps[self.idx], self.offs[n]
        return [w[0] + o[0], w[1] + o[1], w[2] + o[2]]

    def tick(self):
        if self.idx >= len(self.wps):
            return
        for n in self.names:
            g = PoseStamped()
            g.header.stamp = self.get_clock().now().to_msg()
            g.pose.position.x, g.pose.position.y, g.pose.position.z = self.target(n)
            g.pose.orientation.w = 1.0
            self.pubs[n].publish(g)
        if len(self.pos) < len(self.names):
            return
        ok = all(math.dist([self.pos[n].x, self.pos[n].y, self.pos[n].z], self.target(n)) < self.th
                 for n in self.names)
        now = self.get_clock().now().nanoseconds * 1e-9
        if not ok:
            self.reached_t = None
        elif self.reached_t is None:
            self.reached_t = now
        elif now - self.reached_t > self.hold:
            self.idx += 1
            self.reached_t = None
            if self.idx >= len(self.wps) and self.loop:
                self.idx = 0
            self.get_logger().info(f'Waypoint {self.idx}/{len(self.wps)}')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SwarmSequencer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
