#!/usr/bin/env python3
"""
OmniFLY position controller (one node per drone). Replaces position_controller_1..4.py.

In : /<name>/pose (PoseStamped, from mocap / ArUco / any external localisation)
     /<name>/goal (PoseStamped) target position (+yaw)
     /omnifly/emergency (Empty)
Out: /<name>/rc_cmd (Twist), /<name>/arm (Bool), /<name>/pose_rel (PoseStamped)
Srv: /<name>/takeoff, /<name>/land (std_srvs/Trigger)

Output is normalised stick commands for Betaflight ANGLE mode.
GAINS ARE STARTING POINTS: tune with props off / on a tether first.
"""
import csv
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger

IDLE, ARMING, TAKEOFF, FLY, LAND = 'IDLE', 'ARMING', 'TAKEOFF', 'FLY', 'LAND'


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')
        P = self.declare_parameter
        P('drone_name', 'drone1')
        P('rate_hz', 50.0)
        P('use_origin_offset', False)   # True = first pose becomes (0,0,0) like the old code
        P('takeoff_height', 0.5)
        P('climb_rate', 0.25)
        P('descend_rate', 0.15)
        P('arm_delay', 1.0)
        P('pose_timeout', 0.3)
        P('hover_throttle', 0.35)       # throttle that holds altitude: calibrate!
        P('kp_xy', 0.5); P('kd_xy', 0.35); P('max_tilt', 0.25)
        P('kp_z', 0.6); P('ki_z', 0.15); P('kd_z', 0.3)
        P('kp_yaw', 0.8); P('max_yaw', 0.4)
        P('vel_filter', 0.8)
        P('log_dir', '/tmp/omnifly_logs')

        g = lambda k: self.get_parameter(k).value
        self.p = {k: g(k) for k in [
            'takeoff_height', 'climb_rate', 'descend_rate', 'arm_delay', 'pose_timeout',
            'hover_throttle', 'kp_xy', 'kd_xy', 'max_tilt', 'kp_z', 'ki_z', 'kd_z',
            'kp_yaw', 'max_yaw', 'vel_filter', 'use_origin_offset']}
        self.name = g('drone_name')
        self.dt = 1.0 / g('rate_hz')
        self.log_dir = g('log_dir')

        self.state = IDLE
        self.have_pose = False
        self.origin = None
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.last_pose_t = None
        self.goal = None            # (x, y, z, yaw)
        self.hold = [0.0, 0.0, 0.0, 0.0]
        self.z_ref = 0.0
        self.int_z = 0.0
        self.takeoff_z = 0.5
        self.state_t0 = self.get_clock().now()
        self.log = []

        n = self.name
        self.create_subscription(PoseStamped, f'/{n}/pose', self.on_pose, 50)
        self.create_subscription(PoseStamped, f'/{n}/goal', self.on_goal, 10)
        self.create_subscription(Empty, '/omnifly/emergency', self.on_emergency, 10)
        self.cmd_pub = self.create_publisher(Twist, f'/{n}/rc_cmd', 10)
        self.arm_pub = self.create_publisher(Bool, f'/{n}/arm', 10)
        self.rel_pub = self.create_publisher(PoseStamped, f'/{n}/pose_rel', 10)
        self.create_service(Trigger, f'/{n}/takeoff', self.srv_takeoff)
        self.create_service(Trigger, f'/{n}/land', self.srv_land)
        self.create_timer(self.dt, self.loop)
        self.get_logger().info(f'Position controller ready for {n}')

    # ---------- inputs ----------
    def on_pose(self, msg):
        p = msg.pose.position
        raw = [p.x, p.y, p.z]
        if self.origin is None:
            self.origin = raw[:] if self.p['use_origin_offset'] else [0.0, 0.0, 0.0]
            self.get_logger().info(f'Origin set to {self.origin}')
        new = [raw[i] - self.origin[i] for i in range(3)]
        now = self.get_clock().now()
        if self.last_pose_t is not None:
            dt = (now - self.last_pose_t).nanoseconds * 1e-9
            if dt > 1e-3:
                a = self.p['vel_filter']
                self.vel = [a * self.vel[i] + (1 - a) * (new[i] - self.pos[i]) / dt
                            for i in range(3)]
        self.pos, self.last_pose_t, self.have_pose = new, now, True
        self.yaw = yaw_of(msg.pose.orientation)
        rel = PoseStamped()
        rel.header = msg.header
        rel.pose.position.x, rel.pose.position.y, rel.pose.position.z = new
        rel.pose.orientation = msg.pose.orientation
        self.rel_pub.publish(rel)

    def on_goal(self, msg):
        p = msg.pose.position
        self.goal = (p.x, p.y, p.z, yaw_of(msg.pose.orientation))

    def on_emergency(self, _):
        self.get_logger().error('EMERGENCY: disarm')
        self.goal = None
        self.set_state(IDLE)

    def srv_takeoff(self, _, res):
        ok = self.state == IDLE and self.have_pose
        if ok:
            self.set_state(ARMING)
        res.success, res.message = ok, 'arming' if ok else 'need IDLE state and pose'
        return res

    def srv_land(self, _, res):
        ok = self.state in (TAKEOFF, FLY)
        if ok:
            self.set_state(LAND)
        res.success, res.message = ok, 'landing' if ok else 'not flying'
        return res

    def set_state(self, s):
        self.get_logger().info(f'{self.state} -> {s}')
        self.state = s
        self.state_t0 = self.get_clock().now()

    # ---------- control ----------
    def control(self, tx, ty, tz, tyaw):
        p = self.p
        ex, ey = tx - self.pos[0], ty - self.pos[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        bx, by = c * ex + s * ey, -s * ex + c * ey                  # error in body frame
        vbx = c * self.vel[0] + s * self.vel[1]
        vby = -s * self.vel[0] + c * self.vel[1]
        pitch = clamp(p['kp_xy'] * bx - p['kd_xy'] * vbx, -p['max_tilt'], p['max_tilt'])
        roll = clamp(-(p['kp_xy'] * by - p['kd_xy'] * vby), -p['max_tilt'], p['max_tilt'])
        ez = tz - self.pos[2]
        self.int_z = clamp(self.int_z + p['ki_z'] * ez * self.dt, -0.15, 0.15)
        thr = clamp(p['hover_throttle'] + p['kp_z'] * ez + self.int_z - p['kd_z'] * self.vel[2],
                    0.0, 0.9)
        # Betaflight: yaw stick right = clockwise = negative yaw angle (ENU), hence the minus
        yaw = clamp(-p['kp_yaw'] * wrap(tyaw - self.yaw), -p['max_yaw'], p['max_yaw'])
        return pitch, roll, thr, yaw

    def loop(self):
        p, now = self.p, self.get_clock().now()
        in_state = (now - self.state_t0).nanoseconds * 1e-9
        pose_ok = self.have_pose and \
            (now - self.last_pose_t).nanoseconds * 1e-9 < p['pose_timeout']
        if self.state in (ARMING, TAKEOFF, FLY) and not pose_ok:
            self.get_logger().error('Pose lost -> landing')
            self.set_state(LAND)

        arm, pitch, roll, thr, yaw = False, 0.0, 0.0, 0.0, 0.0
        tgt = self.hold[:]

        if self.state == ARMING:
            arm = True
            if in_state > p['arm_delay']:
                self.hold = [self.pos[0], self.pos[1], 0.0, self.yaw]
                self.z_ref, self.int_z = max(self.pos[2], 0.0), 0.0
                # target height: from the last goal if one was given (API takeoff)
                self.takeoff_z = self.goal[2] if (self.goal and self.goal[2] > 0.1) \
                    else p['takeoff_height']
                self.set_state(TAKEOFF)
        elif self.state == TAKEOFF:
            arm = True
            self.z_ref = min(self.z_ref + p['climb_rate'] * self.dt, self.takeoff_z)
            tgt = [self.hold[0], self.hold[1], self.z_ref, self.hold[3]]
            pitch, roll, thr, yaw = self.control(*tgt)
            if self.z_ref >= self.takeoff_z and abs(self.takeoff_z - self.pos[2]) < 0.07:
                self.hold[2] = self.takeoff_z
                self.set_state(FLY)
        elif self.state == FLY:
            arm = True
            tgt = list(self.goal) if self.goal else self.hold
            pitch, roll, thr, yaw = self.control(*tgt)
        elif self.state == LAND:
            arm = True
            self.z_ref = max(self.z_ref - p['descend_rate'] * self.dt, 0.0)
            if pose_ok:
                tgt = [self.hold[0], self.hold[1], self.z_ref, self.hold[3]]
                pitch, roll, thr, yaw = self.control(*tgt)
            else:
                thr = 0.85 * p['hover_throttle']          # blind descent
            if (pose_ok and self.pos[2] < 0.05) or in_state > 15.0:
                self.set_state(IDLE)
                self.goal = None
                arm, thr = False, 0.0

        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z = pitch, roll, thr, yaw
        self.cmd_pub.publish(cmd)
        self.arm_pub.publish(Bool(data=arm))
        self.log.append([now.nanoseconds * 1e-9, self.state, *self.pos, *tgt[:3],
                         roll, pitch, thr, yaw])

    def save_log(self):
        os.makedirs(self.log_dir, exist_ok=True)
        path = os.path.join(self.log_dir, f'{self.name}.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'state', 'x', 'y', 'z', 'x_des', 'y_des', 'z_des',
                        'roll', 'pitch', 'throttle', 'yaw'])
            w.writerows(self.log)
        self.get_logger().info(f'Log saved to {path}')


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm_pub.publish(Bool(data=False))     # disarm on Ctrl+C
        node.save_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
