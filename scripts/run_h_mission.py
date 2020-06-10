#!/usr/bin/env python
import rospy
from MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
import time

def run():
    h_detect_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)
    #h_detect_subscriber = rospy.Subscriber("", Vector3Stamped, h_detect_callback)
    rospy.init_node("head")
    mav = MAV(1)
    height = 5
    
    """ ARM AND TAKE OFF"""
    mav.takeoff(height)
    
    #(0, 0, 2) -> (0, 1, 2)
    # t=0
    # while t != 100:
    #     mav.set_position(0, t/100, 2)
    #     mav.rate.sleep()
    mav.set_position(0, 1, height)

    """ INITIALIZE H DETECTION """
    for i in range(10):
        h_detect_publisher.publish(Bool(True))

    # while not terminou or timout:
    #     rate.sleep()
    init_time = time.time()
    while not time.time() - init_time >= 10:
        mav.set_position(0, 1, height)
        mav.rate.sleep()
        
    h_detect_publisher.publish(Bool(False))
    
    """ LAND """

    """ DISARM """
    mav.RTL()

if __name__ == "__main__":
    run()
    