#!/usr/bin/env python
import rospy
from MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
running_state = False
def running_callback(bool):
    global running_state
    running_state = bool.data

def run():
    global running_state
    rospy.init_node("head")
    cv_control_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)
    running_sub = rospy.Subscriber("/cv_detection/set_running_state", Bool, running_callback)
    mav = MAV("1")
    height = 2
    for i in range(10):
        rospy.logwarn("Deactivating CV Control")
        cv_control_publisher.publish(Bool(False))
        mav.rate.sleep()

    """ ARM AND TAKE OFF"""
    mav.takeoff(height)
    height = 2

    mav.set_position(0, 0, height)

    """ INITIALIZE CV_CONTROL """
    for i in range (10):
        cv_control_publisher.publish(Bool(True))
        mav.rate.sleep()


    init_time = rospy.get_rostime().secs
    while running_state == 1 and rospy.get_rostime().secs - init_time <= 120:
        running_sub
        #rospy.logwarn("Precision Landing...")
        #mav.set_vel(0, 0, 0)
        mav.rate.sleep()

    """ END CV CONTROL """
    '''for i in range(10):
        rospy.logwarn("Deactivating CV Control")
        cv_control_publisher.publish(Bool(False))
        mav.rate.sleep()'''

    """ LAND AND DISARM """
    if(running_state == 0):
        rospy.logwarn("Landing Drone")
        mav.land()
    else:
        mav.RTL()
        rospy.logwarn("RLT")

if __name__ == "__main__":
    run()
    