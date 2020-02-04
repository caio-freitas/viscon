#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty

takeoff = rospy.Publisher('tello/takeoff',Empty, queue_size=10)
land = rospy.Publisher('tello/land', Empty, queue_size=10)
yaw = rospy.Publisher('tello/cmd_vel', Twist,queue_size=10)
rate = rospy.Rate(hz=10)
velocity = Twist()


def set_velocity(w):
    velocity.linear.x = 0
    velocity.linear.y  =  0
    velocity.linear.z  =  0
    velocity.angular.z = w
    yaw.publish(velocity)


if __name__ == '__main__':
    try:
        takeoff.publish(Empty())
        rospy.sleep(10.)
        set_velocity(1)
        rospy.sleep(3.)
        set_velocity(-1)
        rospy.sleep(3.)
        land.publish(Empty())



