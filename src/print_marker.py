#!/usr/bin/env python

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
rospy.init_node("marker_printer")
rate = rospy.Rate(20)

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

marker = Marker()
marker.header.frame_id = "map"
marker.header.stamp = rospy.Time()
marker.ns = "/"
marker.id = 0   
marker.type = Marker.SPHERE
marker.action = Marker.ADD
marker.pose.position.x = -2
marker.pose.position.y = -0.5
marker.pose.position.z = 0
# marker.pose.orientation.x = 0
# marker.pose.orientation.y = 0
# marker.pose.orientation.z = 0
# marker.pose.orientation.w = 1
marker.scale = Vector3(1, 1, 1)
marker.color = ColorRGBA(r=0, g=1, b=0, a=0.9)
marker.lifetime = rospy.Duration(0, 0)
# rate.sleep()
# marker_pub.publish(marker)
# rate.sleep()
t=0
while not rospy.is_shutdown():
    marker.pose.position.x += 0.5*np.sin(t)
    marker.pose.position.y += 0.5*np.cos(t)
    marker.pose.position.z += 0.1
    
    marker_pub.publish(marker)
    t += 0.1
    rate.sleep()