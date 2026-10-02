#!/usr/bin/env python
"""Headless stand-in for joint_state_publisher_gui.

Sweeps every revolute joint in robot_description back and forth so the model's
motion can be checked in Foxglove without an X display on the VM.
"""
import math
import xml.etree.ElementTree as ET

import rospy
from sensor_msgs.msg import JointState


def revolute_joints(urdf):
    """Return [(name, lower, upper)] for each revolute joint, in URDF order."""
    joints = []
    for joint in ET.fromstring(urdf).iter("joint"):
        if joint.get("type") == "revolute":
            limit = joint.find("limit")
            joints.append((joint.get("name"), float(limit.get("lower")), float(limit.get("upper"))))
    return joints


def main():
    rospy.init_node("joint_sweep")
    period = rospy.get_param("~period", 4.0)  # seconds per full sweep
    amplitude = rospy.get_param("~amplitude", 0.5)  # fraction of each joint's half-range
    joints = revolute_joints(rospy.get_param("robot_description"))
    pub = rospy.Publisher("joint_states", JointState, queue_size=1)

    msg = JointState(name=[name for name, _, _ in joints])
    rate = rospy.Rate(30)
    start = rospy.get_time()
    while not rospy.is_shutdown():
        t = rospy.get_time() - start
        msg.position = []
        for i, (_, lower, upper) in enumerate(joints):
            # Offset each joint's phase so the motion ripples around the body leg by leg
            phase = 2 * math.pi * (t / period - float(i) / len(joints))
            msg.position.append((lower + upper) / 2 + amplitude * (upper - lower) / 2 * math.sin(phase))
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
