"""
Python API for OmniFLY drones.

Counterpart of crazyflie_py/crazyflie.py. Instead of the Crazyflie firmware
(high-level commander, onboard trajectories) the setpoints are generated here,
on the PC, and streamed to the position controller of the omnifly package.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
import yaml


def _yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _smooth(u):
    """Minimum-jerk blend, 0..1 -> 0..1 with zero velocity/acceleration at ends."""
    u = min(max(u, 0.0), 1.0)
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5


class TimeHelper:
    """Object containing all time-related functionality."""

    def __init__(self, node):
        self.node = node
        self.rateHz = None
        self.nextTime = None

    def time(self):
        """Return current time in seconds."""
        return self.node.get_clock().now().nanoseconds / 1e9

    def sleep(self, duration):
        """Sleep for the provided duration in seconds (keeps ROS spinning)."""
        end = self.time() + duration
        while rclpy.ok() and self.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0)

    def sleepForRate(self, rateHz):
        """Sleep so that, if called in a loop, executes at specified rate."""
        if self.nextTime is None or self.rateHz != rateHz:
            self.rateHz = rateHz
            self.nextTime = self.time() + 1.0 / rateHz
        while rclpy.ok() and self.time() < self.nextTime:
            rclpy.spin_once(self.node, timeout_sec=0)
        self.nextTime += 1.0 / rateHz

    def isShutdown(self):
        """Return True if the script should abort, e.g. from Ctrl-C."""
        return not rclpy.ok()


class Drone:
    """
    Object representing a single OmniFLY drone.

    Setpoints are published on /<name>/goal; takeoff/land use the services of
    the position controller. Motions are generated here at 50 Hz.
    """

    RATE_HZ = 50.0

    def __init__(self, node, name):
        self.node = node
        self.name = name
        self.prefix = '/' + name
        self._pos = None
        self._yaw = 0.0
        self._setpoint = None   # (np.array(3), yaw) last commanded
        self._motion = None     # callable(t) -> (pos, yaw)
        self._duration = 0.0
        self._t0 = 0.0

        self.takeoffService = node.create_client(Trigger, self.prefix + '/takeoff')
        self.landService = node.create_client(Trigger, self.prefix + '/land')
        self.goalPublisher = node.create_publisher(PoseStamped, self.prefix + '/goal', 10)
        self.emergencyPublisher = node.create_publisher(Empty, '/omnifly/emergency', 10)
        node.create_subscription(
            PoseStamped, self.prefix + '/pose_rel', self._poseCallback, 10)
        node.create_timer(1.0 / self.RATE_HZ, self._tick)

    # ---------- internals ----------
    def _now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def _poseCallback(self, msg):
        p = msg.pose.position
        self._pos = np.array([p.x, p.y, p.z])
        self._yaw = _yaw_of(msg.pose.orientation)

    def _publish(self, pos, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        self.goalPublisher.publish(msg)
        self._setpoint = (np.array(pos, dtype=float), float(yaw))

    def _start(self, motion, duration):
        self._motion = motion
        self._duration = float(duration)
        self._t0 = self._now()

    def _tick(self):
        if self._motion is None:
            return
        t = self._now() - self._t0
        pos, yaw = self._motion(min(t, self._duration))
        self._publish(pos, yaw)
        if t >= self._duration:
            self._motion = None

    def _call(self, client, what):
        if not client.service_is_ready():
            self.node.get_logger().error(f'{self.name}: {what} service not available')
            return

        def done(future):
            res = future.result()
            if res is not None and not res.success:
                self.node.get_logger().warn(f'{self.name}: {what} refused: {res.message}')

        client.call_async(Trigger.Request()).add_done_callback(done)

    def _base(self):
        """Return (position, yaw) to start a new motion from."""
        if self._setpoint is not None:
            return self._setpoint
        return self.position(), self._yaw

    # ---------- public API ----------
    def position(self):
        """Return the current measured position (np.array of 3, meters)."""
        if self._pos is None:
            raise RuntimeError(f'No pose received yet for {self.name}')
        return self._pos.copy()

    def yaw(self):
        """Return the current measured yaw in radians."""
        return self._yaw

    def emergency(self):
        """Emergency stop: disarm all motors immediately (affects ALL drones)."""
        self._motion = None
        self.emergencyPublisher.publish(Empty())

    def takeoff(self, targetHeight, duration, groupMask=0):
        """
        Arm, climb straight up to targetHeight, then hover.

        Asynchronous. The climb speed is set by the controller parameter
        climb_rate, so 'duration' is only kept for API compatibility.
        """
        pos = self.position()
        self._motion = None
        self._publish([pos[0], pos[1], targetHeight], self._yaw)
        self._call(self.takeoffService, 'takeoff')

    def land(self, targetHeight, duration, groupMask=0):
        """
        Descend and disarm. Asynchronous.

        The descent speed is set by the controller parameter descend_rate;
        'targetHeight' and 'duration' are kept for API compatibility.
        """
        self._motion = None
        self._setpoint = None
        self._call(self.landService, 'land')

    def goTo(self, goal, yaw, duration, relative=False, groupMask=0):
        """
        Move smoothly (minimum-jerk) to the goal, then hover. Asynchronous.

        Args:
        ----
            goal (iterable of 3 floats): Goal position in meters.
            yaw (float): Goal yaw in radians.
            duration (float): Time to reach the goal in seconds.
            relative (bool): If True, goal is an offset from the current setpoint.
            groupMask (int): Unused, API compatibility.

        """
        start, start_yaw = self._base()
        goal = np.array(goal, dtype=float)
        if relative:
            goal = start + goal
            yaw = start_yaw + yaw
        start = np.array(start, dtype=float)
        dur = float(duration)

        def motion(t):
            s = _smooth(t / dur) if dur > 0 else 1.0
            return start + (goal - start) * s, start_yaw + (yaw - start_yaw) * s

        self._start(motion, dur)

    def executeTrajectory(self, trajectory, timescale=1.0, relative=True):
        """
        Stream a uav_trajectory.Trajectory from the PC (replaces onboard start).

        Only position and yaw are used; there is no velocity/acceleration
        feedforward on this platform.
        """
        start, _ = self._base()
        shift = np.zeros(3)
        if relative:
            shift = np.array(start, dtype=float) - trajectory.eval(0.0).pos
        dur = trajectory.duration * timescale

        def motion(t):
            e = trajectory.eval(min(t / timescale, trajectory.duration))
            return e.pos + shift, e.yaw

        self._start(motion, dur)

    def cmdPosition(self, pos, yaw=0.):
        """Send a streaming position and yaw setpoint (cancels running goTo)."""
        self._motion = None
        self._publish(pos, yaw)

    def cmdFullState(self, pos, vel, acc, yaw, omega):
        """Streaming setpoint; only pos and yaw are used (no feedforward here)."""
        self.cmdPosition(pos, yaw)


def _enabled_names():
    path = os.path.join(get_package_share_directory('omnifly'), 'config', 'drones.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return [n for n, r in cfg['robots'].items() if r.get('enabled')]


class DroneSwarm:
    """
    Container and broadcaster for all enabled drones.

    Attributes:
        crazyflies (List[Drone]): one object per enabled drone in drones.yaml
            (name kept from Crazyswarm for script compatibility).
        crazyfliesById (Dict[int, Drone]): index by the number in the name.
        crazyfliesByName (Dict[str, Drone]): index by name.

    """

    def __init__(self, node, timeHelper, pose_timeout=10.0):
        self.node = node
        self.crazyflies = []
        self.crazyfliesById = {}
        self.crazyfliesByName = {}
        for name in _enabled_names():
            d = Drone(node, name)
            self.crazyflies.append(d)
            self.crazyfliesByName[name] = d
            digits = ''.join(c for c in name if c.isdigit())
            self.crazyfliesById[int(digits) if digits else len(self.crazyflies)] = d
        self.drones = self.crazyflies
        self.emergencyPublisher = node.create_publisher(Empty, '/omnifly/emergency', 10)
        self._wait_for_poses(timeHelper, pose_timeout)

    def _wait_for_poses(self, timeHelper, timeout):
        end = timeHelper.time() + timeout
        while rclpy.ok() and timeHelper.time() < end:
            if all(d._pos is not None for d in self.crazyflies):
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        missing = [d.name for d in self.crazyflies if d._pos is None]
        self.node.get_logger().warn(f'No pose received for: {missing}')

    def emergency(self):
        """Emergency stop of all drones."""
        for d in self.crazyflies:
            d._motion = None
        self.emergencyPublisher.publish(Empty())

    def takeoff(self, targetHeight, duration, groupMask=0):
        """Broadcast takeoff to all drones."""
        for d in self.crazyflies:
            d.takeoff(targetHeight, duration)

    def land(self, targetHeight, duration, groupMask=0):
        """Broadcast landing to all drones."""
        for d in self.crazyflies:
            d.land(targetHeight, duration)

    def goTo(self, goal, yaw, duration, groupMask=0):
        """Broadcast goTo; the goal is an offset relative to each drone."""
        for d in self.crazyflies:
            d.goTo(goal, yaw, duration, relative=True)
