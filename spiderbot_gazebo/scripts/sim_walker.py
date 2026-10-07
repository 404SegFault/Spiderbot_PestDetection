#!/usr/bin/env python
"""Pretend walking for the simulated Spiderbot.

Leg physics isn't simulated. Instead this node takes velocity commands on /cmd_vel
(linear.x forward, linear.y sideways, angular.z turning), moves the robot over the
terrain by setting its pose in Gazebo each tick, tilts the body to match the ground
under its feet, and animates a tripod gait so it looks like walking. It publishes what
the real robot's drivers would:

  /odom, TF odom -> base_footprint   odometry with drift, as when feet slip
  /joint_states                      leg angles for the gait animation

plus, for the simulator only, /ground_truth/odom (the true pose in the world frame,
for scoring) and /gazebo/set_model_state. The odom frame starts at the spawn pose.

Safety: the robot stops if no command arrives for ~cmd_timeout seconds, and won't walk
toward anything /scan sees closer than ~stop_distance in its direction of travel.
"""
import math
import random
import struct
import xml.etree.ElementTree as ET

import numpy as np
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Header


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                      cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class Terrain(object):
    """Ground height from a heightfield mesh on a regular grid, as generate_crawlspace.py
    writes. Flat (z = 0) when no mesh is given."""

    def __init__(self, stl_path):
        self.z = None
        if not stl_path:
            return
        with open(stl_path, "rb") as f:
            data = f.read()
        heights = {}
        for i in range(struct.unpack_from("<I", data, 80)[0]):
            v = struct.unpack_from("<12f", data, 84 + 50 * i)
            for k in (3, 6, 9):
                heights[(round(v[k], 3), round(v[k + 1], 3))] = v[k + 2]
        xs = sorted(set(x for x, _ in heights))
        ys = sorted(set(y for _, y in heights))
        self.x0, self.y0, self.res = xs[0], ys[0], xs[1] - xs[0]
        self.z = np.array([[heights.get((x, y), 0.0) for x in xs] for y in ys])

    def height(self, x, y):
        if self.z is None:
            return 0.0
        ny, nx = self.z.shape
        fx, fy = (x - self.x0) / self.res, (y - self.y0) / self.res
        i, j = int(min(max(math.floor(fx), 0), nx - 2)), int(min(max(math.floor(fy), 0), ny - 2))
        tx, ty = min(max(fx - i, 0.0), 1.0), min(max(fy - j, 0.0), 1.0)
        z = self.z
        return float(z[j, i] * (1 - tx) * (1 - ty) + z[j, i + 1] * tx * (1 - ty)
                     + z[j + 1, i] * (1 - tx) * ty + z[j + 1, i + 1] * tx * ty)


def leg_geometry(urdf):
    """Each leg's joint-name prefix, mount position and yaw, and zero-pose foot position,
    relative to base_footprint, worked out from the URDF."""
    root = ET.fromstring(urdf)
    parent_joint = {j.find("child").get("link"): j for j in root.findall("joint")}

    def pose(link):  # 4x4 pose of a link in base_footprint at zero joint angles
        joint = parent_joint.get(link)
        if joint is None:
            return np.eye(4)
        origin = joint.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
        local = np.eye(4)
        local[:3, :3], local[:3, 3] = rpy_matrix(*rpy), xyz
        return pose(joint.find("parent").get("link")).dot(local)

    legs = []
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name.endswith("_coxa_joint"):
            prefix = name[:-len("_coxa_joint")]
            mount, foot = pose(prefix + "_coxa_link"), pose(prefix + "_foot_link")[:3, 3]
            legs.append({
                "prefix": prefix,
                "yaw": math.atan2(mount[1, 0], mount[0, 0]),
                "foot": foot,
                "reach": math.hypot(foot[0] - mount[0, 3], foot[1] - mount[1, 3]),
                "drop": mount[2, 3] - foot[2],
                "angle": math.atan2(mount[1, 3], mount[0, 3]),
            })
    # Tripod gait: alternate legs going round the body move together
    for k, leg in enumerate(sorted(legs, key=lambda l: l["angle"])):
        leg["phase_offset"] = math.pi * (k % 2)
    return legs


def clamp(value, limit):
    return max(-limit, min(limit, value))


class Walker(object):
    def __init__(self):
        p = rospy.get_param
        self.model_name = p("~model_name", "spiderbot")
        self.max_linear, self.max_angular = p("~max_linear", 0.15), p("~max_angular", 0.6)
        self.max_accel, self.max_angular_accel = p("~max_accel", 0.4), p("~max_angular_accel", 1.5)
        self.cmd_timeout = p("~cmd_timeout", 0.4)
        self.stop_distance = p("~stop_distance", 0.3)
        self.step_frequency, self.lift = p("~step_frequency", 1.5), p("~lift", 0.35)
        # Odometry drift (set ~odom_noise false for perfect odometry): strides come out
        # slightly short, turns slightly over-reported, plus a random walk in heading.
        self.odom_noise = p("~odom_noise", True)
        self.scale_error, self.turn_error = p("~odom_scale_error", -0.03), p("~odom_turn_error", 0.03)
        self.yaw_noise = p("~odom_yaw_noise", 0.05)  # rad per sqrt(metre walked)
        self.rng = random.Random(p("~seed", 1))

        self.terrain = Terrain(p("~terrain_mesh", ""))
        self.legs = leg_geometry(p("robot_description"))
        self.joint_names = [l["prefix"] + s for l in self.legs for s in ("_coxa_joint", "_femur_joint", "_tibia_joint")]

        self.x, self.y, self.yaw = p("~spawn_x", 0.0), p("~spawn_y", 0.0), p("~spawn_yaw", 0.0)  # true, world frame
        self.ox = self.oy = self.oyaw = 0.0  # odometry, odom frame (origin at the spawn pose)
        self.v = [0.0, 0.0, 0.0]  # current body velocity: forward, left, turn
        self.cmd, self.cmd_time = [0.0, 0.0, 0.0], -1e9
        self.scan, self.scan_angles, self.scan_time = None, None, -1e9
        self.phase = 0.0
        self.z, self.roll, self.pitch = self.ground_pose()

        self.tf = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
        self.truth_pub = rospy.Publisher("ground_truth/odom", Odometry, queue_size=5)
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=5)
        self.gazebo_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        rospy.Subscriber("cmd_vel", Twist, self.on_cmd, queue_size=1)
        rospy.Subscriber("scan", LaserScan, self.on_scan, queue_size=1)

    def on_cmd(self, msg):
        self.cmd = [clamp(msg.linear.x, self.max_linear), clamp(msg.linear.y, self.max_linear),
                    clamp(msg.angular.z, self.max_angular)]
        self.cmd_time = rospy.get_time()

    def on_scan(self, msg):
        if self.scan_angles is None or len(self.scan_angles) != len(msg.ranges):
            self.scan_angles = msg.angle_min + msg.angle_increment * np.arange(len(msg.ranges))
        self.scan, self.scan_time = msg, rospy.get_time()

    def blocked(self, vx, vy):
        """Is something within stop_distance in the direction of travel? (The lidar
        faces the same way as the body.)"""
        if self.stop_distance <= 0 or math.hypot(vx, vy) < 1e-3 or rospy.get_time() - self.scan_time > 1.0:
            return False
        ranges = np.asarray(self.scan.ranges)
        off = np.abs((self.scan_angles - math.atan2(vy, vx) + math.pi) % (2 * math.pi) - math.pi)
        near = (off < math.radians(35)) & (ranges > self.scan.range_min) & (ranges < self.stop_distance)
        return bool(near.any())

    def ground_pose(self):
        """Body height, roll and pitch from a plane through the ground under the feet."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        a = np.array([[1.0, l["foot"][0], l["foot"][1]] for l in self.legs])
        h = np.array([self.terrain.height(self.x + l["foot"][0] * c - l["foot"][1] * s,
                                          self.y + l["foot"][0] * s + l["foot"][1] * c) for l in self.legs])
        z, slope_x, slope_y = np.linalg.lstsq(a, h, rcond=-1)[0]  # rcond=-1 works on numpy 1.13 (Ubuntu 18.04)
        return float(z), math.atan(slope_y), -math.atan(slope_x)

    def gait(self, dt):
        vx, vy, wz = self.v
        activity = min(1.0, (math.hypot(vx, vy) + 0.12 * abs(wz)) / 0.02)
        if activity > 0.01:
            self.phase = (self.phase + 2 * math.pi * self.step_frequency * dt) % (2 * math.pi)
        quarter = 1.0 / (4 * self.step_frequency)  # a stride is half a cycle long
        positions = []
        for leg in self.legs:
            fx, fy = leg["foot"][0], leg["foot"][1]
            # Velocity of the body over this foot, split along and across the leg
            fvx, fvy = vx - wz * fy, vy + wz * fx
            along = -fvx * math.sin(leg["yaw"]) + fvy * math.cos(leg["yaw"])
            outward = fvx * math.cos(leg["yaw"]) + fvy * math.sin(leg["yaw"])
            ph = self.phase + leg["phase_offset"]
            stroke = math.cos(ph)  # +1 front of stride .. -1 back; stance is the first half-cycle
            lift = activity * self.lift * max(0.0, -math.sin(ph))  # lifted during the swing
            positions += [clamp(along * quarter * stroke / leg["reach"], math.pi / 4),
                          clamp(lift, math.pi / 2),
                          clamp(outward * quarter * stroke / leg["drop"] - lift, math.pi / 2)]
        return positions

    def step(self, dt):
        now = rospy.get_time()
        target = list(self.cmd) if now - self.cmd_time < self.cmd_timeout else [0.0, 0.0, 0.0]
        if self.blocked(target[0], target[1]):
            target[0] = target[1] = 0.0
        for k, accel in ((0, self.max_accel), (1, self.max_accel), (2, self.max_angular_accel)):
            self.v[k] += clamp(target[k] - self.v[k], accel * dt)
        vx, vy, wz = self.v

        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.yaw += wz * dt

        dyaw, scale = wz * dt, 1.0
        if self.odom_noise:
            dyaw = dyaw * (1 + self.turn_error) + self.rng.gauss(0, self.yaw_noise * math.sqrt(math.hypot(vx, vy) * dt))
            scale = 1 + self.scale_error
        oc, os_ = math.cos(self.oyaw), math.sin(self.oyaw)
        self.ox += (vx * oc - vy * os_) * dt * scale
        self.oy += (vx * os_ + vy * oc) * dt * scale
        self.oyaw += dyaw

        z, roll, pitch = self.ground_pose()
        smooth = min(1.0, 8.0 * dt)  # ease the body over bumps
        self.z += (z - self.z) * smooth
        self.roll += (roll - self.roll) * smooth
        self.pitch += (pitch - self.pitch) * smooth
        self.publish(rospy.Time.now(), self.gait(dt))

    def publish(self, stamp, joint_positions):
        vx, vy, wz = self.v
        truth_q = quaternion(self.roll, self.pitch, self.yaw)
        odom_q = quaternion(self.roll, self.pitch, self.oyaw)

        state = ModelState(model_name=self.model_name, reference_frame="world")
        state.pose.position.x, state.pose.position.y, state.pose.position.z = self.x, self.y, self.z
        state.pose.orientation = truth_q
        self.gazebo_pub.publish(state)

        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "odom", "base_footprint"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.ox, self.oy, self.z
        t.transform.rotation = odom_q
        self.tf.sendTransform(t)

        for pub, frame, (x, y), q in ((self.odom_pub, "odom", (self.ox, self.oy), odom_q),
                                      (self.truth_pub, "world", (self.x, self.y), truth_q)):
            msg = Odometry()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, frame, "base_footprint"
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, self.z
            msg.pose.pose.orientation = q
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = vx, vy, wz
            pub.publish(msg)

        self.joint_pub.publish(JointState(header=Header(stamp=stamp), name=self.joint_names, position=joint_positions))


def main():
    rospy.init_node("sim_walker")
    walker = Walker()
    rate = rospy.Rate(rospy.get_param("~rate", 25))
    last = rospy.get_time()
    while not rospy.is_shutdown():
        now = rospy.get_time()
        dt = now - last
        if 0 < dt < 0.5:
            walker.step(dt)
        last = now
        try:
            rate.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            last = rospy.get_time()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
