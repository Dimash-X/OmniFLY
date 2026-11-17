#!/usr/bin/env python3
"""
Hardware interface: ROS 2 -> ELRS TX module (CRSF) -> drone ELRS receiver.
Replaces the Crazyflie server for ONE drone. Run one instance per drone.

Subscribes:  /<name>/rc_cmd (Twist)  linear.x=pitch, linear.y=roll  [-1..1]
                                      linear.z=throttle [0..1], angular.z=yaw [-1..1]
             /<name>/arm    (Bool)
Channels (AETR): 1 roll, 2 pitch, 3 throttle, 4 yaw, 5 ARM (AUX1), 6 ANGLE mode (AUX2)
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from omnifly import crsf


class CrsfBridge(Node):
    def __init__(self):
        super().__init__('crsf_bridge')
        self.declare_parameter('drone_name', 'drone1')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 400000)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('cmd_timeout', 0.3)
        self.declare_parameter('dry_run', False)  # log instead of sending

        name = self.get_parameter('drone_name').value
        self.timeout = self.get_parameter('cmd_timeout').value
        self.dry_run = self.get_parameter('dry_run').value
        self.ser = None
        if not self.dry_run:
            import serial
            self.ser = serial.Serial(self.get_parameter('serial_port').value,
                                     self.get_parameter('baud').value, timeout=0)

        self.cmd = Twist()
        self.armed = False
        self.last_cmd = 0.0
        self.create_subscription(Twist, f'/{name}/rc_cmd', self.on_cmd, 10)
        self.create_subscription(Bool, f'/{name}/arm', self.on_arm, 10)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.send)
        self.get_logger().info(f'CRSF bridge for {name} (dry_run={self.dry_run})')

    def on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd = time.monotonic()

    def on_arm(self, msg):
        self.armed = msg.data

    def send(self):
        alive = (time.monotonic() - self.last_cmd) < self.timeout
        c = self.cmd if alive else Twist()           # failsafe: zero throttle
        armed = self.armed and alive
        ch = [
            crsf.norm_to_crsf(c.linear.y),           # roll
            crsf.norm_to_crsf(c.linear.x),           # pitch
            crsf.unit_to_crsf(c.linear.z),           # throttle
            crsf.norm_to_crsf(c.angular.z),          # yaw
            crsf.CH_MAX if armed else crsf.CH_MIN,   # AUX1 arm
            crsf.CH_MAX,                             # AUX2 angle mode
        ]
        frame = crsf.rc_frame(ch)
        if self.dry_run:
            self.get_logger().info(f'ch={ch}', throttle_duration_sec=1.0)
        else:
            self.ser.write(frame)

    def destroy_node(self):
        if self.ser:
            for _ in range(10):  # make sure we disarm
                self.ser.write(crsf.rc_frame([crsf.CH_MID, crsf.CH_MID, crsf.CH_MIN,
                                              crsf.CH_MID, crsf.CH_MIN, crsf.CH_MAX]))
                time.sleep(0.01)
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CrsfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
