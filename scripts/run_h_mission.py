#!/usr/bin/env python
import rospy
from MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
import time

def run():

    rospy.init_node("head")
    mav = MAV(1)
    height = 2
    
    h_detect_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)

    cv_control_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)

    """ ARM AND TAKE OFF"""
    mav.takeoff(height)
    height = 2


    #(0, 0, 2) -> (0, 1, 2)
    # t=0 1, 2)
    # t=0
    # while t != 100:
    #     mav.set_position(0, t/100, 2)
    #     mav.rate.sleep()
    mav.set_position(0, 1, height)

    """ INITIALIZE CV_CONTROL """
    for i in range (10):
        cv_control_publisher.publish(Bool(True))
        h_detect_publisher.publish(Bool(True))
        mav.rate.sleep()
        

    # while not terminou or timout:
    #     rate.sleep()

    init_time = time.time()
    while not time.time() - init_time >= 10:
        #mav.set_vel(0, 0, 0)
        mav.rate.sleep()

    """ END H DETECTION """
    h_detect_publisher.publish(Bool(False))
    
    """ END CV CONTROL """
    h_detect_publisher.publish(Bool(False))

    """ LAND AND DISARM """
    mav.RTL()

if __name__ == "__main__":
    run()
    