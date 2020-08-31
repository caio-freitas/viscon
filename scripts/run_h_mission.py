#!/usr/bin/env python
import rospy
from MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped

def run():

    rospy.init_node("head")
    cv_control_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)
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

    # while not terminou or timout:
    #     rate.sleep()

    init_time = rospy.get_rostime().secs
    while rospy.get_rostime().secs - init_time <= 20:
        #mav.set_vel(0, 0, 0)
        mav.rate.sleep()

    """ END CV CONTROL """
    for i in range(10):
        rospy.logwarn("Deactivating CV Control")
        cv_control_publisher.publish(Bool(False))
        mav.rate.sleep()

    """ LAND AND DISARM """
    mav.RTL()

if __name__ == "__main__":
    run()
    