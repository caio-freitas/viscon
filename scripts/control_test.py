#!/usr/bin/env python

import rospy
from MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped

def go():
    rospy.init_node("head")
    mav = MAV("1")
    height = 2
    
    mav.takeoff(height)
    mav.set_position(0, 0, height)

    mav.rate.sleep()    
    for x in range (100):
        rospy.logwarn("SETTING POSITION")
        mav.set_position(3, 3, height)
    
    mav.land()
    mav._disarm()


if __name__ == "__main__":
    go()


