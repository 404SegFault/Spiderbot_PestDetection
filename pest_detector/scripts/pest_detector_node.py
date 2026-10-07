#!/usr/bin/env python
"""Find pest signs in the robot's camera and locate them in 3D.

Runs a detector (pest_detector.rules.RuleBasedDetector; anything with the same
detect(bgr) -> [Detection] interface can replace it) on the colour camera a few times a
second, and works out where each sign is from the depth image. The depth camera can't
measure closer than ~0.6 m, so for signs on the ground closer than that it uses where the
pixel's ray meets the ground instead. Signs at the wrong height are dropped: ground signs
must be on the ground, and termite tubes (which need real depth) between the ground and
0.65 m up, which rules out things like overhead pipes.

Publishes:
  /pest_detection/detections        vision_msgs/Detection2DArray, header = the image's.
                                     results[0].id = class (1 termite_mud_tube, 2 rodent_droppings,
                                     3 rodent_burrow), .score, .pose = 3D position in the camera frame
  /pest_detection/image/compressed  the camera frame with detections drawn on it (JPEG)
"""
import math

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from pest_detector.rules import LABELS, RuleBasedDetector

GROUND_SIGNS = ("rodent_droppings", "rodent_burrow")
COLORS = {"termite_mud_tube": (0, 140, 255), "rodent_droppings": (255, 0, 255), "rodent_burrow": (0, 230, 255)}
SHORT_NAMES = {"termite_mud_tube": "termite tube", "rodent_droppings": "droppings", "rodent_burrow": "burrow"}


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class PestDetectorNode(object):
    def __init__(self):
        self.detector = RuleBasedDetector()
        self.period = 1.0 / rospy.get_param("~rate", 3.0)  # detection runs this often (Hz), not every frame
        self.min_depth, self.max_depth = rospy.get_param("~min_depth", 0.6), rospy.get_param("~max_depth", 8.0)
        self.ground_frame = rospy.get_param("~ground_frame", "base_footprint")
        self.bridge = CvBridge()
        self.camera = None  # fx, fy, cx, cy
        self.last = 0.0
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf)

        self.det_pub = rospy.Publisher("pest_detection/detections", Detection2DArray, queue_size=5)
        self.img_pub = rospy.Publisher("pest_detection/image/compressed", CompressedImage, queue_size=2)
        rospy.Subscriber("camera/rgb/camera_info", CameraInfo, self.on_info, queue_size=1)
        rgb = message_filters.Subscriber("camera/rgb/image_raw", Image, queue_size=2, buff_size=2 ** 24)
        depth = message_filters.Subscriber("camera/depth/image_raw", Image, queue_size=2, buff_size=2 ** 24)
        message_filters.ApproximateTimeSynchronizer([rgb, depth], 4, 0.05).registerCallback(self.on_images)

    def on_info(self, msg):
        self.camera = (msg.K[0], msg.K[4], msg.K[2], msg.K[5])

    def on_images(self, rgb_msg, depth_msg):
        now = rospy.get_time()
        if self.camera is None or now - self.last < self.period:
            return
        self.last = now
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg).astype(np.float32)
        if depth_msg.encoding == "16UC1":  # millimetres on the real Astra
            depth /= 1000.0
        try:
            ground_tf = self.tf.lookup_transform(self.ground_frame, rgb_msg.header.frame_id, rgb_msg.header.stamp,
                                                 rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        out = Detection2DArray(header=rgb_msg.header)
        for det in self.detector.detect(bgr):
            point = self.locate(det, depth, ground_tf)
            if point is None:
                continue
            d = Detection2D(header=rgb_msg.header)
            x, y, w, h = det.box
            d.bbox.center.x, d.bbox.center.y, d.bbox.size_x, d.bbox.size_y = x + w / 2.0, y + h / 2.0, w, h
            hyp = ObjectHypothesisWithPose(id=LABELS.index(det.label) + 1, score=det.score)
            hyp.pose.pose.position.x, hyp.pose.pose.position.y, hyp.pose.pose.position.z = point
            hyp.pose.pose.orientation.w = 1.0
            d.results.append(hyp)
            out.detections.append(d)
            self.draw(bgr, det)
        self.det_pub.publish(out)
        jpeg = CompressedImage(header=rgb_msg.header, format="jpeg")
        jpeg.data = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
        self.img_pub.publish(jpeg)

    def locate(self, det, depth, ground_tf):
        """3D position (camera frame) of the detection's point, or None."""
        fx, fy, cx, cy = self.camera
        u, v = det.point
        patch = depth[max(v - 2, 0):v + 3, max(u - 2, 0):u + 3]
        valid = patch[np.isfinite(patch) & (patch >= self.min_depth) & (patch <= self.max_depth)]
        ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])  # optical frame: z forward, x right, y down
        rot = quat_matrix(ground_tf.transform.rotation)
        origin = np.array([ground_tf.transform.translation.x, ground_tf.transform.translation.y,
                           ground_tf.transform.translation.z])
        if valid.size:
            point = ray * float(np.median(valid))
        elif det.label not in GROUND_SIGNS:
            return None  # only ground signs can be placed without depth
        else:
            # Too close for the depth camera: use where the ray meets the ground
            direction = rot.dot(ray)
            if direction[2] >= -1e-3:
                return None  # looking at or above the horizon
            t = -origin[2] / direction[2]
            if t * math.sqrt(ray.dot(ray)) > self.min_depth + 0.1:
                return None  # depth should have seen something this far away: it's beyond range or the sky
            point = ray * t
        height = rot.dot(point)[2] + origin[2]
        if det.label in GROUND_SIGNS and abs(height) > 0.1:
            return None  # a "ground" sign that isn't on the ground
        if det.label == "termite_mud_tube" and not 0.0 < height < 0.65:
            return None  # tubes climb walls from the ground; this is something else (e.g. a pipe overhead)
        return tuple(float(c) for c in point)

    def draw(self, bgr, det):
        x, y, w, h = det.box
        color = COLORS[det.label]
        cv2.rectangle(bgr, (x - 2, y - 2), (x + w + 2, y + h + 2), color, 2)
        text = "%s %.0f%%" % (SHORT_NAMES[det.label], det.score * 100)
        cv2.putText(bgr, text, (x, max(14, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(bgr, text, (x, max(14, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


if __name__ == "__main__":
    rospy.init_node("pest_detector")
    PestDetectorNode()
    rospy.spin()
