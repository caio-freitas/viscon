#!/usr/bin/env python

import rospy
from viscon.MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped

def go():
    rospy.init_node("head")
    mav = MAV("1")
    height = 4
    
    mav.takeoff(height)
    mav.rate.sleep()
    mav.set_position(0, 0, height)
    mav.rate.sleep()
    
    for x in range (150):
        rospy.logwarn("SETTING POSITION")
        mav.set_position(3, 3, height)
        mav.rate.sleep()
    
    mav.set_altitude(1)
    rospy.logwarn("PEGUE A SUA ENCOMENDA")
    mav.set_altitude(height)

    for y in range (100):
        rospy.logwarn("GOING BACK")
        mav.set_position(0, 0, height)
        mav.rate.sleep()

    mav.rate.sleep()
    mav.RTL()
    #mav.land()
    #mav._disarm()


if __name__ == "__main__":
    go()


