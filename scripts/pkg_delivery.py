#!/usr/bin/env python
import rospy
from MAV import MAV

#MASK_POSITION = 0b00000000011, // x, y, z, vyaw
#0b110111111000
MASK_POSITION = 0b0000111111111100

def run():

    rospy.init_node("head")
    mav = MAV("1")


    # while mav.global_pose.latitude < 0.1:
    #     mav.rate.sleep()


    height = 5
    altitude = 1

    mav.takeoff(height)
    mav.rate.sleep()
    for i in range(20):
        mav.rate.sleep()
        
    lat_init = mav.global_pose.latitude
    lon_init = mav.global_pose.longitude
    lat = lat_init + 0.00002
    lon = lon_init + 0.00008

    rospy.logerr("{},{}".format(lat, lon))
                                
    mav.go_gps_target(type_mask=MASK_POSITION, lat=lat, lon=lon)

    mav.set_altitude(altitude)
    mav.rate.sleep()
    rospy.logwarn('CHEGUEI')
    
    mav.set_altitude(height)
    mav.rate.sleep()

    mav.go_gps_target(MASK_POSITION, lat_init, lon_init)
    mav.rate.sleep()

    mav.RTL()

if __name__ == "__main__":
    run()
    
    

    