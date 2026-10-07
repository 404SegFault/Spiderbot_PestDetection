#!/usr/bin/env python
"""Build a soil moisture map from probe readings, on the SLAM map.

Each /moisture reading (sensor_msgs/RelativeHumidity, frame = probe tip) is placed where
TF says the probe is in the map frame, i.e. where SLAM thinks the robot is, and averaged
into a grid. Publishes /moisture_map (nav_msgs/OccupancyGrid): 1 (dry) to 98 (saturated),
0 = not measured. Foxglove's Costmap colour mode draws that as blue -> red with
unmeasured cells transparent. The grid is drawn ~height above the map plane so terrain
bumps don't hide it.

Service /save_maps (std_srvs/Trigger) saves this map and the latest /map (SLAM floor
plan) as map_server-format PGM + YAML files in a new timestamped folder under ~save_dir.
"""
import os
import time

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import RelativeHumidity
from std_srvs.srv import Trigger, TriggerResponse


def write_pgm(path, rows):
    """rows: 2-D uint8 array, first row = top of the image."""
    with open(path, "wb") as f:
        f.write(("P5\n%d %d\n255\n" % (rows.shape[1], rows.shape[0])).encode("ascii"))
        f.write(rows.astype(np.uint8).tobytes())


def write_map_yaml(path, image, resolution, origin, mode, comment):
    with open(path, "w") as f:
        f.write("# %s\nimage: %s\nresolution: %s\norigin: [%s, %s, 0.0]\n" % (
            comment, image, resolution, origin[0], origin[1]))
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: %s\n" % mode)


class MoistureMapper(object):
    def __init__(self):
        p = rospy.get_param
        self.frame = p("~map_frame", "map")
        self.res = p("~resolution", 0.1)
        size = p("~size", 20.0)  # metres, square, centred on the map origin
        self.radius = p("~radius", 0.12)  # each reading fills cells within this distance
        self.height = p("~height", 0.12)
        self.save_dir = os.path.expanduser(p("~save_dir", "~/spiderbot_maps"))
        self.n = int(round(size / self.res))
        self.origin = (-size / 2.0, -size / 2.0)
        self.total = np.zeros((self.n, self.n))
        self.count = np.zeros((self.n, self.n))
        self.readings = 0
        self.floor_plan = None

        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf)
        self.pub = rospy.Publisher("moisture_map", OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber("moisture", RelativeHumidity, self.on_reading, queue_size=20)
        rospy.Subscriber("map", OccupancyGrid, self.on_floor_plan, queue_size=1)
        rospy.Service("save_maps", Trigger, self.save)
        rospy.Timer(rospy.Duration(1.0 / p("~publish_rate", 1.0)), self.publish)

    def on_floor_plan(self, msg):
        self.floor_plan = msg

    def on_reading(self, msg):
        try:
            t = self.tf.lookup_transform(self.frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return  # SLAM not running yet
        x, y = t.transform.translation.x, t.transform.translation.y
        value = msg.relative_humidity * 100.0
        r = int(np.ceil(self.radius / self.res))
        ci, cj = int((x - self.origin[0]) / self.res), int((y - self.origin[1]) / self.res)
        for j in range(max(cj - r, 0), min(cj + r + 1, self.n)):
            for i in range(max(ci - r, 0), min(ci + r + 1, self.n)):
                cx, cy = self.origin[0] + (i + 0.5) * self.res, self.origin[1] + (j + 0.5) * self.res
                if (cx - x) ** 2 + (cy - y) ** 2 <= self.radius ** 2:
                    self.total[j, i] += value
                    self.count[j, i] += 1
        self.readings += 1

    def grid_values(self):
        """Mean moisture per cell, 1-98, with 0 where nothing was measured (row 0 = bottom)."""
        mean = np.divide(self.total, self.count, out=np.zeros_like(self.total), where=self.count > 0)
        return np.where(self.count > 0, np.clip(np.round(mean), 1, 98), 0).astype(np.int8)

    def publish(self, _event):
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = rospy.Time.now(), self.frame
        grid.info.resolution, grid.info.width, grid.info.height = self.res, self.n, self.n
        grid.info.origin.position.x, grid.info.origin.position.y = self.origin
        grid.info.origin.position.z = self.height
        grid.info.origin.orientation.w = 1.0
        grid.data = self.grid_values().flatten().tolist()
        self.pub.publish(grid)

    def save(self, _request):
        folder = os.path.join(self.save_dir, time.strftime("%Y-%m-%d_%H-%M-%S"))
        os.makedirs(folder)
        write_pgm(os.path.join(folder, "moisture.pgm"), np.flipud(self.grid_values().astype(np.uint8)))
        write_map_yaml(os.path.join(folder, "moisture.yaml"), "moisture.pgm", self.res, self.origin, "raw",
                       "Measured soil moisture, 1 (dry) to 98 (saturated), 0 = not measured; map frame")
        saved = ["moisture map (%d readings)" % self.readings]
        if self.floor_plan is not None:
            info = self.floor_plan.info
            cells = np.array(self.floor_plan.data, dtype=np.int16).reshape(info.height, info.width)
            # Same encoding as map_server's map_saver: occupied black, free white, unknown grey
            image = np.full(cells.shape, 205, dtype=np.uint8)
            image[(cells >= 0) & (cells <= 25)] = 254
            image[cells >= 65] = 0
            write_pgm(os.path.join(folder, "map.pgm"), np.flipud(image))
            write_map_yaml(os.path.join(folder, "map.yaml"), "map.pgm", info.resolution,
                           (info.origin.position.x, info.origin.position.y), "trinary", "SLAM floor plan; map frame")
            saved.append("floor plan")
        message = "Saved %s to %s" % (" and ".join(saved), folder)
        rospy.loginfo(message)
        return TriggerResponse(success=True, message=message)


if __name__ == "__main__":
    rospy.init_node("moisture_mapper")
    MoistureMapper()
    rospy.spin()
