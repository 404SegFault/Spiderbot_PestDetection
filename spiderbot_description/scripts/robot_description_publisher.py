#!/usr/bin/env python
"""Republish the robot_description parameter as a latched /robot_description topic.

Foxglove can't read ROS params over a rosbridge connection, but its 3D panel can
load a URDF from a std_msgs/String topic.
"""
import rospy
from std_msgs.msg import String

rospy.init_node("robot_description_publisher")
pub = rospy.Publisher("robot_description", String, queue_size=1, latch=True)
pub.publish(String(data=rospy.get_param("robot_description")))
rospy.spin()
