import rclpy
import rclpy.node

from . import genericJoystick
from .drone import DroneSwarm, TimeHelper


class OmniFly:
    """Entry point of the OmniFLY Python API (counterpart of Crazyswarm)."""

    def __init__(self):
        """Init ROS 2, create the swarm object and wait for all drone poses."""
        rclpy.init()
        self.node = rclpy.node.Node('OmniFlyAPI')
        self.timeHelper = TimeHelper(self.node)
        self.allcfs = DroneSwarm(self.node, self.timeHelper)
        self.input = genericJoystick.Joystick(self.timeHelper)
