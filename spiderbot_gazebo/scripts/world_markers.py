#!/usr/bin/env python
"""Publish a Gazebo world's visuals as a latched MarkerArray for Foxglove.

Gazebo's GUI can't open on the headless VM and Foxglove can't read .world files,
so this mirrors the world's box / cylinder / sphere / mesh visuals as markers in
the `world` frame. Each Gazebo model becomes one marker namespace, so models can
be toggled individually in Foxglove's 3D panel.

Limitations: meshes must be binary STL, <include>d models and material scripts
are skipped, and every <pose> is treated as relative to its parent element.
"""
import math
import os
import struct
import xml.etree.ElementTree as ET

import rospy
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))  # (position, quaternion xyzw)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def compose(parent, child):
    """Pose of `child` (given relative to `parent`) in parent's frame."""
    (px, py, pz), pq = parent
    cp, cq = child
    rx, ry, rz, _ = quat_mul(quat_mul(pq, (cp[0], cp[1], cp[2], 0.0)), (-pq[0], -pq[1], -pq[2], pq[3]))
    return ((px + rx, py + ry, pz + rz), quat_mul(pq, cq))


def local_pose(element):
    text = element.findtext("pose")
    if not text:
        return IDENTITY
    x, y, z, roll, pitch, yaw = [float(v) for v in text.split()]
    return ((x, y, z), quat_from_rpy(roll, pitch, yaw))


def visual_color(visual):
    for tag in ("material/diffuse", "material/ambient"):
        text = visual.findtext(tag)
        if text:
            return ColorRGBA(*[float(v) for v in text.split()])
    return ColorRGBA(0.7, 0.7, 0.7, 1.0)


def resolve_uri(uri, search_paths):
    if uri.startswith("file://"):
        return uri[len("file://"):]
    if uri.startswith("model://"):
        for directory in search_paths:
            path = os.path.join(directory, uri[len("model://"):])
            if os.path.isfile(path):
                return path
    return None


def load_stl(path, scale):
    """Triangle vertices from a binary STL file."""
    with open(path, "rb") as f:
        data = f.read()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError("%s is not a binary STL file" % path)
    sx, sy, sz = scale
    points = []
    for i in range(count):
        v = struct.unpack_from("<12f", data, 84 + 50 * i)
        points += [Point(v[k] * sx, v[k + 1] * sy, v[k + 2] * sz) for k in (3, 6, 9)]
    return points


def geometry_marker(geometry, search_paths):
    """Marker with type and scale set from an SDF <geometry>, or None if unsupported."""
    shape = geometry[0]
    marker = Marker()
    if shape.tag == "box":
        marker.type = Marker.CUBE
        marker.scale.x, marker.scale.y, marker.scale.z = [float(v) for v in shape.findtext("size").split()]
    elif shape.tag == "cylinder":
        marker.type = Marker.CYLINDER
        marker.scale.x = marker.scale.y = 2 * float(shape.findtext("radius"))
        marker.scale.z = float(shape.findtext("length"))
    elif shape.tag == "sphere":
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = 2 * float(shape.findtext("radius"))
    elif shape.tag == "mesh":
        uri = shape.findtext("uri")
        path = resolve_uri(uri, search_paths)
        if path is None:
            rospy.logwarn("Skipping mesh %s: not found in %s", uri, search_paths)
            return None
        marker.type = Marker.TRIANGLE_LIST
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.points = load_stl(path, [float(v) for v in (shape.findtext("scale") or "1 1 1").split()])
    else:
        rospy.logwarn("Skipping unsupported <%s> geometry", shape.tag)
        return None
    return marker


def world_markers(world_file, frame_id, search_paths, hidden_models):
    world = ET.parse(world_file).getroot().find("world")
    markers = []

    def add_model(model, parent_pose, ns):
        pose = compose(parent_pose, local_pose(model))
        for link in model.findall("link"):
            link_pose = compose(pose, local_pose(link))
            for visual in link.findall("visual"):
                marker = geometry_marker(visual.find("geometry"), search_paths)
                if marker is None:
                    continue
                (x, y, z), q = compose(link_pose, local_pose(visual))
                marker.header.frame_id = frame_id
                marker.ns = ns
                marker.id = len(markers)
                marker.frame_locked = True
                marker.pose.position = Point(x, y, z)
                marker.pose.orientation = Quaternion(*q)
                marker.color = visual_color(visual)
                markers.append(marker)
        for nested in model.findall("model"):
            add_model(nested, pose, ns)

    for model in world.findall("model"):
        if model.get("name") not in hidden_models:
            add_model(model, IDENTITY, model.get("name"))
    if world.findall("include"):
        rospy.logwarn("Skipping %d <include>d models", len(world.findall("include")))
    return markers


def main():
    rospy.init_node("world_markers")
    world_file = rospy.get_param("~world")
    search_paths = rospy.get_param("~model_path", "").split(":") + os.environ.get("GAZEBO_MODEL_PATH", "").split(":")
    hidden = [name.strip() for name in rospy.get_param("~hide_models", "").split(",")]
    markers = world_markers(world_file, rospy.get_param("~frame_id", "world"), [p for p in search_paths if p], hidden)

    pub = rospy.Publisher("world_markers", MarkerArray, queue_size=1, latch=True)
    pub.publish(MarkerArray(markers=markers))
    rospy.loginfo("Published %d markers from %s", len(markers), world_file)
    rospy.spin()


if __name__ == "__main__":
    main()
