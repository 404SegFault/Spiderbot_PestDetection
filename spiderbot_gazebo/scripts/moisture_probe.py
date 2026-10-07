#!/usr/bin/env python
"""Simulated soil moisture probe.

Gazebo has no moisture sensor, so this reads the crawlspace's ground-truth moisture map
at the probe tip's true position (from /ground_truth/odom and the moisture_probe_link
frame), adds noise, and publishes it on /moisture as sensor_msgs/RelativeHumidity:
relative_humidity is relative soil moisture from 0.01 (dry) to 0.98 (saturated), and the
header frame is the probe tip.
"""
import math
import os
import random

import numpy as np
import rospy
import tf2_ros
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import RelativeHumidity


def load_moisture_map(yaml_path):
    """The map_server-format moisture map: (values[row, col] with row 0 at the bottom, resolution, origin x, y)."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(os.path.dirname(yaml_path), meta["image"]), "rb") as f:
        data = f.read()
    fields = data.split(None, 4)  # P5, width, height, maxval, pixels
    width, height = int(fields[1]), int(fields[2])
    pixels = np.frombuffer(fields[4][-width * height:], dtype=np.uint8).reshape(height, width)
    return np.flipud(pixels).astype(float), float(meta["resolution"]), meta["origin"][0], meta["origin"][1]


class MoistureProbe(object):
    def __init__(self):
        self.values, self.res, self.x0, self.y0 = load_moisture_map(rospy.get_param("~truth_map"))
        self.noise = rospy.get_param("~noise", 2.0)  # standard deviation, moisture units (1-98)
        self.frame = rospy.get_param("~probe_frame", "moisture_probe_link")
        self.rng = random.Random(rospy.get_param("~seed", 2))

        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf)
        offset = buf.lookup_transform("base_footprint", self.frame, rospy.Time(0), rospy.Duration(30)).transform.translation
        self.offset = (offset.x, offset.y)
        self.pose = None

        self.pub = rospy.Publisher("moisture", RelativeHumidity, queue_size=5)
        rospy.Subscriber("ground_truth/odom", Odometry, self.on_pose, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / rospy.get_param("~rate", 5.0)), self.read)

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def read(self, _event):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        px = x + self.offset[0] * math.cos(yaw) - self.offset[1] * math.sin(yaw)
        py = y + self.offset[0] * math.sin(yaw) + self.offset[1] * math.cos(yaw)
        row, col = int((py - self.y0) / self.res), int((px - self.x0) / self.res)
        if not (0 <= row < self.values.shape[0] and 0 <= col < self.values.shape[1]):
            return
        value = min(max(self.values[row, col] + self.rng.gauss(0, self.noise), 1.0), 98.0)
        msg = RelativeHumidity(relative_humidity=value / 100.0, variance=(self.noise / 100.0) ** 2)
        msg.header.stamp, msg.header.frame_id = rospy.Time.now(), self.frame
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("moisture_probe")
    MoistureProbe()
    rospy.spin()
