#!/usr/bin/env python
"""Turn per-frame pest detections into confirmed signs on the map, rate them, and report.

Each detection from /pest_detection/detections is placed on the SLAM map and merged with
earlier sightings of the same kind of sign nearby. A sign is confirmed once it's been seen
~confirm times, which filters out one-off false detections. Confirmed signs are combined
with the measured soil moisture (/moisture_map) into a risk rating:

  termite mud tube   HIGH in damp soil (termites need moisture), otherwise MEDIUM
  rodent burrow      HIGH (an active entry point)
  rodent droppings   MEDIUM

Publishes /pest_detection/signs (visualization_msgs/MarkerArray, map frame): a disc per
confirmed sign coloured by risk (red HIGH, orange MEDIUM) and a label.

Service /pest_detection/save_report (std_srvs/Trigger): saves the maps (via /save_maps) and
writes report.md + signs.json beside them. In simulation, if ~ground_truth (the world's
answer key) is set, the report also scores the run against it.
"""
import json
import math
import os
import time

import numpy as np
import rospy
import tf2_ros
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from std_srvs.srv import Trigger, TriggerResponse
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

LABELS = ("termite_mud_tube", "rodent_droppings", "rodent_burrow")
NAMES = {"termite_mud_tube": "Termite mud tube", "rodent_droppings": "Rodent droppings", "rodent_burrow": "Rodent burrow"}
# Sightings of the same kind within this distance (m) are one sign. Tubes stay tight: they
# can be under half a metre apart along a wall.
MERGE_RADIUS = {"termite_mud_tube": 0.3, "rodent_droppings": 0.5, "rodent_burrow": 0.4}
# Confirmed signs of the same kind this close (m) are reported as one: a sign's position
# drifts as sightings average in, so twins can survive the merge above.
CONSOLIDATE_RADIUS = {"termite_mud_tube": 0.3, "rodent_droppings": 0.75, "rodent_burrow": 0.6}
MOISTURE_RADIUS = 0.6  # probe readings this close to a sign count as its soil moisture
DAMP = 45  # moisture (1-98) at which soil counts as damp
RISK_COLORS = {"HIGH": (0.9, 0.15, 0.15), "MEDIUM": (1.0, 0.6, 0.1)}
# Answer-key types the detector can't see: the camera is tilted down and the joists are overhead
OUT_OF_VIEW = {"damaged_wood": "overhead joists are above the camera's view"}
LOOK_ALIKE = {"mud_smear": "termite_mud_tube", "pebbles": "rodent_droppings", "water_stain": "damaged_wood"}


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def apply(transform, point):
    t = transform.transform
    return quat_matrix(t.rotation).dot(point) + np.array([t.translation.x, t.translation.y, t.translation.z])


class SignMapper(object):
    def __init__(self):
        p = rospy.get_param
        self.frame = p("~map_frame", "map")
        self.confirm = p("~confirm", 3)
        self.report_dir = os.path.expanduser(p("~report_dir", "~/spiderbot_maps"))
        truth_path = p("~ground_truth", "")
        self.truth = yaml.safe_load(open(truth_path))["signs"] if truth_path else None
        self.truth_frame = p("~truth_frame", "world")
        self.signs = []  # dicts: label, pos (map frame), n, score, first, last
        self.moisture = None
        self.path = []  # true robot positions (simulation only), for "closest approach" in the report

        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf)
        self.pub = rospy.Publisher("pest_detection/signs", MarkerArray, queue_size=1, latch=True)
        rospy.Subscriber("pest_detection/detections", Detection2DArray, self.on_detections, queue_size=10)
        rospy.Subscriber("moisture_map", OccupancyGrid, self.on_moisture, queue_size=1)
        if self.truth is not None:
            rospy.Subscriber("ground_truth/odom", Odometry, self.on_truth_pose, queue_size=1)
        rospy.Service("pest_detection/save_report", Trigger, self.save_report)
        rospy.Timer(rospy.Duration(1.0), self.publish)

    def on_moisture(self, msg):
        self.moisture = msg

    def on_truth_pose(self, msg):
        p = msg.pose.pose.position
        if not self.path or math.hypot(p.x - self.path[-1][0], p.y - self.path[-1][1]) > 0.05:
            self.path.append((p.x, p.y))

    def on_detections(self, msg):
        try:
            to_map = self.tf.lookup_transform(self.frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        for d in msg.detections:
            r = d.results[0]
            label = LABELS[r.id - 1]
            p = r.pose.pose.position
            pos = apply(to_map, np.array([p.x, p.y, p.z]))
            near = [s for s in self.signs if s["label"] == label
                    and math.hypot(s["pos"][0] - pos[0], s["pos"][1] - pos[1]) <= MERGE_RADIUS[label]]
            if near:
                s = min(near, key=lambda s: math.hypot(s["pos"][0] - pos[0], s["pos"][1] - pos[1]))
                s["pos"] = (s["pos"] * s["n"] + pos) / (s["n"] + 1)
                s["n"] += 1
                s["score"] = max(s["score"], r.score)
                s["last"] = msg.header.stamp.to_sec()
            else:
                self.signs.append({"label": label, "pos": pos, "n": 1, "score": r.score,
                                   "first": msg.header.stamp.to_sec(), "last": msg.header.stamp.to_sec()})

    def confirmed(self):
        signs = []
        for s in sorted((s for s in self.signs if s["n"] >= self.confirm), key=lambda s: -s["n"]):
            twin = next((k for k in signs if k["label"] == s["label"] and math.hypot(
                k["pos"][0] - s["pos"][0], k["pos"][1] - s["pos"][1]) <= CONSOLIDATE_RADIUS[s["label"]]), None)
            if twin is None:
                signs.append(dict(s))
            else:  # fold into the more-seen twin, weighted by sightings
                twin["pos"] = (twin["pos"] * twin["n"] + s["pos"] * s["n"]) / (twin["n"] + s["n"])
                twin["n"] += s["n"]
                twin["score"] = max(twin["score"], s["score"])
                twin["first"] = min(twin["first"], s["first"])
        burrows = [s["pos"] for s in signs if s["label"] == "rodent_burrow"]
        # From further away a burrow's hole can break up into specks that look like droppings
        return [s for s in signs if not (s["label"] == "rodent_droppings" and any(
            math.hypot(s["pos"][0] - b[0], s["pos"][1] - b[1]) < 0.3 for b in burrows))]

    def moisture_near(self, x, y, radius=MOISTURE_RADIUS):
        """Mean measured moisture within radius of (x, y) on the map, or None if none measured."""
        m = self.moisture
        if m is None:
            return None
        info = m.info
        grid = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
        i0, j0 = int((x - info.origin.position.x) / info.resolution), int((y - info.origin.position.y) / info.resolution)
        r = int(math.ceil(radius / info.resolution))
        patch = grid[max(j0 - r, 0):j0 + r + 1, max(i0 - r, 0):i0 + r + 1]
        measured = patch[patch > 0]
        return float(measured.mean()) if measured.size else None

    @staticmethod
    def risk(label, moisture):
        if label == "termite_mud_tube":
            return "HIGH" if moisture is not None and moisture >= DAMP else "MEDIUM"
        return "HIGH" if label == "rodent_burrow" else "MEDIUM"

    def findings(self):
        out = []
        for s in self.confirmed():
            moisture = self.moisture_near(s["pos"][0], s["pos"][1])
            out.append(dict(s, moisture=moisture, risk=self.risk(s["label"], moisture)))
        return sorted(out, key=lambda f: (f["risk"] != "HIGH", f["first"]))

    def publish(self, _event):
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for k, f in enumerate(self.findings()):
            x, y, z = f["pos"]
            color = RISK_COLORS[f["risk"]]
            disc = Marker(type=Marker.CYLINDER, ns="sign", id=k, frame_locked=True)
            disc.header.frame_id = self.frame
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, z
            disc.pose.orientation.w = 1.0
            disc.scale.x, disc.scale.y, disc.scale.z = 0.22, 0.22, 0.02
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = color + (0.9,)
            label = Marker(type=Marker.TEXT_VIEW_FACING, ns="label", id=k, frame_locked=True)
            label.header.frame_id = self.frame
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, z + 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.09
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            moisture = "moisture %d" % f["moisture"] if f["moisture"] is not None else "moisture ?"
            label.text = "%s | %s | %s" % (NAMES[f["label"]], f["risk"], moisture)
            markers.markers += [disc, label]
        self.pub.publish(markers)

    def save_report(self, _request):
        folder = None
        try:
            rospy.wait_for_service("save_maps", timeout=2.0)
            message = rospy.ServiceProxy("save_maps", Trigger)().message
            folder = message.rsplit(" to ", 1)[-1] if " to " in message else None
        except (rospy.ROSException, rospy.ServiceException):
            pass
        if not folder or not os.path.isdir(folder):  # mapping isn't running: report on its own
            folder = os.path.join(self.report_dir, time.strftime("%Y-%m-%d_%H-%M-%S"))
            os.makedirs(folder)
        findings = self.findings()
        lines = ["# Spiderbot inspection report", "",
                 "Saved %s. Positions are metres in the map frame (origin = where the robot started)." %
                 time.strftime("%Y-%m-%d %H:%M"), "", "## Findings", ""]
        if findings:
            lines += ["| # | Sign | Risk | Position (x, y) | Soil moisture within %.1f m | Seen |" % MOISTURE_RADIUS,
                      "|---|---|---|---|---|---|"]
            for k, f in enumerate(findings):
                lines.append("| %d | %s | %s | %.2f, %.2f | %s | %d times |" % (
                    k + 1, NAMES[f["label"]], f["risk"], f["pos"][0], f["pos"][1],
                    "%.0f" % f["moisture"] if f["moisture"] is not None else "not measured: walk the probe closer",
                    f["n"]))
        else:
            lines.append("No pest signs confirmed.")
        lines += ["", "Risk: termite tubes are HIGH in damp soil (moisture %d+ on the 1-98 scale), otherwise MEDIUM;"
                  " burrows are HIGH; droppings are MEDIUM." % DAMP, ""]
        lines += self.moisture_summary()
        if self.truth is not None:
            lines += self.scoring(findings)
        with open(os.path.join(folder, "report.md"), "w") as f:
            f.write("\n".join(lines) + "\n")
        with open(os.path.join(folder, "signs.json"), "w") as f:
            json.dump([{"type": s["label"], "risk": s["risk"], "x": round(s["pos"][0], 3), "y": round(s["pos"][1], 3),
                        "z": round(s["pos"][2], 3), "moisture": s["moisture"], "sightings": s["n"],
                        "confidence": round(s["score"], 2)} for s in findings], f, indent=1)
        message = "Saved report (%d signs) to %s" % (len(findings), folder)
        rospy.loginfo(message)
        return TriggerResponse(success=True, message=message)

    def moisture_summary(self):
        if self.moisture is None:
            return []
        values = np.array(self.moisture.data, dtype=np.int16)
        values = values[values > 0]
        if not values.size:
            return []
        area = values.size * self.moisture.info.resolution ** 2
        return ["## Soil moisture", "",
                "Measured %.1f m2 along the robot's path: mean %.0f, wettest %d; %.1f m2 damp (%d+)." % (
                    area, values.mean(), values.max(), np.sum(values >= DAMP) * self.moisture.info.resolution ** 2, DAMP),
                ""]

    def scoring(self, findings):
        try:
            to_world = self.tf.lookup_transform(self.truth_frame, self.frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return ["## Scoring", "", "Couldn't score: no transform from %s to %s." % (self.frame, self.truth_frame)]
        found = [(f["label"], apply(to_world, f["pos"])) for f in findings]

        def detected(kind, x, y, radius=0.5):
            return [p for label, p in found if label == kind and math.hypot(p[0] - x, p[1] - y) <= radius]

        def closest(x, y):
            return min(math.hypot(px - x, py - y) for px, py in self.path) if self.path else float("nan")

        real = [t for t in self.truth if not t["decoy"]]
        detectable = [t for t in real if t["type"] not in OUT_OF_VIEW]
        hits = [t for t in detectable if detected(t["type"], t["x"], t["y"])]
        decoys = [t for t in self.truth if t["decoy"]]
        fooled = [t for t in decoys if detected(LOOK_ALIKE.get(t["type"], ""), t["x"], t["y"])]
        false_alarms = [(label, p) for label, p in found if not any(
            t["type"] == label and math.hypot(t["x"] - p[0], t["y"] - p[1]) <= 0.5 for t in real)]
        lines = ["## Scoring against the simulation's answer key", "",
                 "- Found **%d of %d** detectable pest signs." % (len(hits), len(detectable)),
                 "- False alarms: **%d**." % len(false_alarms),
                 "- Decoys ignored: **%d of %d**." % (len(decoys) - len(fooled), len(decoys)), "",
                 "| Answer-key sign | Where | Result | Robot's closest approach |", "|---|---|---|---|"]
        for t in self.truth:
            if t["decoy"]:
                result = "decoy, wrongly reported" if t in fooled else "decoy, correctly ignored"
            elif t["type"] in OUT_OF_VIEW:
                result = "not detectable (%s)" % OUT_OF_VIEW[t["type"]]
            else:
                result = "**found**" if t in hits else "missed"
            lines.append("| %s | %s | %s | %.1f m |" % (t["type"], t["where"], result, closest(t["x"], t["y"])))
        for label, p in false_alarms:
            lines.append("| (false alarm) %s | %.2f, %.2f in world | reported, but nothing there | |" % (label, p[0], p[1]))
        return lines + [""]


if __name__ == "__main__":
    rospy.init_node("sign_mapper")
    SignMapper()
    rospy.spin()
