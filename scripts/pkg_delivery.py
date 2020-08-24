#!/usr/bin/env python
import rospy
from MAV import MAV

def run():

    rospy.init_node("head")
    mav = MAV("1")

    lat_init = mav.global_pose.latitude
    lon_init = mav.global_pose.longitude
    # ANTES TAVA + 0.0005
    lat = 47.40
    lon = 8.55

    height = 5
    mav.takeoff(height)
    mav.rate.sleep()
    
    mav.set_global_target(lat, lon)

    altitude = 1
    mav.lower_altitude(altitude)
    mav.rate.sleep()
    rospy.logwarn('CHEGUEI')
    
    mav.takeoff(height)
    mav.rate.sleep()

    mav.set_global_target(lat_init, lon_init)
    mav.rate.sleep()

    mav.RTL()

if __name__ == "__main__":
    run()
    
    

    