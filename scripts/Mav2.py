#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import Mavlink
import time
import math

TOL = 0.5
MAX_TIME_DISARM = 15
CONFIG = {"mavros_local_position_pub" : "/mavros/setpoint_position/local",
                "mavros_velocity_pub" : "/mavros/setpoint_velocity/cmd_vel",
                "mavros_local_atual" :  "/mavros/local_position/pose",
                "mavros_state_sub" :    "/mavros/state",
                "mavros_arm" :          "/mavros/cmd/arming",
                "mavros_set_mode" :     "/mavros/set_mode",
                "mavros_battery_sub" :  "/mavros/battery"}
class MAV:
    
                #"bebop_velocity_pub" : "/bebop/setpoint_velocity/cmd_vel"}

    def __init__(self, mav_name, mav_type="mavros"):
        rospy.init_node("MAV_" + mav_name)
        self.rate = rospy.Rate(60)

        self.drone_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.goal_vel = TwistStamped()
        self.drone_state = State()
        self.battery = BatteryState()
        # ############## Publishers ##############
        self.local_position_pub = rospy.Publisher(CONFIG[mav_type + "_local_position_pub"], PoseStamped, queue_size = 20)
        self.velocity_pub = rospy.Publisher(CONFIG[mav_type + "_velocity_pub"],  TwistStamped, queue_size=5)

        ################### Swarm ###########
        # self.local_position_pub = rospy.Publisher(mav_name + "/setpoint_position/local", PoseStamped, queue_size = 20)
        # self.velocity_pub = rospy.Publisher(mav_name + "/setpoint_velocity/cmd_vel",  TwistStamped, queue_size=5)

        #self.mavlink_pub = rospy.Publisher('/mavlink/to', Mavlink, queue_size=1)
        ########## Subscribers ##################
        self.local_atual = rospy.Subscriber(CONFIG[mav_type + "_local_atual"], PoseStamped, self.local_callback)
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback) #'mavros/state' maybe?
        self.battery_sub = rospy.Subscriber(CONFIG[mav_type + "_battery_sub"], BatteryState, self.battery_callback)
        self.extended_state_sub = rospy.Subscriber("/mavros/extended_status", ExtendedState, self.extended_state_callback, queue_size=2)
        ############# Services ##################
        self.arm = rospy.ServiceProxy(CONFIG[mav_type + "_arm"], CommandBool)
        self.set_mode_srv= rospy.ServiceProxy('mavros/set_mode', SetMode)

        self.LAND_STATE = ExtendedState.LANDED_STATE_UNDEFINED # landing state
        '''
        0: Undefined
        1: On ground
        2: In air
        '''
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")


    ###### Callback Functions ##########
    def state_callback(self, state_data):
        print(state_data)
        print("state callback")
        self.drone_state = state_data

    def battery_callback(self, bat_data):
        self.battery = bat_data

    def local_callback(self, local):
        self.drone_pose.pose.position.x = local.pose.position.x
        self.drone_pose.pose.position.y = local.pose.position.y
        self.drone_pose.pose.position.z = local.pose.position.z

    def extended_state_callback(self, es_data):
        self.LAND_STATE = es_data.landed_state

    ####### Set Position and Velocity ################
    def set_position(self, x, y, z):
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = z
        self.local_position_pub.publish(self.goal_pose)
        self.rate.sleep()

    def set_vel(self, x, y, z, roll=0, pitch=0, yaw=0):
        self.goal_vel.twist.linear.x = x
        self.goal_vel.twist.linear.y = y
        self.goal_vel.twist.linear.z = z

        self.goal_vel.twist.angular.x = roll
        self.goal_vel.twist.angular.y = pitch
        self.goal_vel.twist.angular.z = yaw
        self.velocity_pub.publish(self.goal_vel)


    def set_mode(self, mode, timeout): 
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.drone_state.mode
        loop_freq = 1  # Hz
        loop_rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in range(timeout * loop_freq):
            if self.drone_state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                loop_rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    def chegou(self):
        if (abs(self.goal_pose.pose.position.x - self.drone_pose.pose.position.x) < TOL) and (abs(self.goal_pose.pose.position.y - self.drone_pose.pose.position.y) < TOL) and (abs(self.goal_pose.pose.position.z - self.drone_pose.pose.position.z) < TOL):
            return True
        else:
            return False

    def takeoff(self, height):
        velocity = 1.0
        part = velocity/60.0
        inicial_height = (0.8 * 60 * part) / velocity 
        for i in range(100):
            self.set_position(self.drone_pose.pose.position.x, self.drone_pose.pose.position.y, 0)
            self.rate.sleep()


        rospy.logwarn("ARMING DRONE")
        self.arm(True)
        rospy.loginfo("DRONE ARMED")  
        self.rate.sleep()

        t=0
        t += inicial_height
        while not rospy.is_shutdown() and self.drone_pose.pose.position.z <= height:

            if self.drone_state.mode != "OFFBOARD":
                rospy.loginfo("SETTING OFFBOARD FLIGHT MODE")
                self.set_mode("OFFBOARD", 5) 

            if t < height:
                rospy.logwarn('TAKING OFF AT ' + str(velocity) + ' m/s')
                self.set_position(self.drone_pose.pose.position.x, self.drone_pose.pose.position.y, t)
                t += part
            else:
                self.set_position(self.drone_pose.pose.position.x, self.drone_pose.pose.position.y, height)

            rospy.loginfo('Position: (' + str(self.drone_pose.pose.position.x) + ', ' + str(self.drone_pose.pose.position.y) + ', '+ str(self.drone_pose.pose.position.z) + ')')
            self.rate.sleep()

        self.set_position(self.drone_pose.pose.position.x, self.drone_pose.pose.position.y, height)

        return "done"

    def RTL(self):
        velocity = 0.3
        ds = velocity/60.0

        self.rate.sleep()
        height = self.drone_pose.pose.position.z
        rospy.loginfo('Position: (' + str(self.drone_pose.pose.position.x) + ', ' + str(self.drone_pose.pose.position.y) + ', ' + str(self.drone_pose.pose.position.z) + ')')

        self.set_position(0,0,height)
        self.rate.sleep()
        rospy.loginfo('Position: (' + str(self.drone_pose.pose.position.x) + ', ' + str(self.drone_pose.pose.position.y) + ', ' + str(self.drone_pose.pose.position.z) + ')')
        rospy.loginfo('Goal Position: (' + str(self.goal_pose.pose.position.x) + ', ' + str(self.goal_pose.pose.position.y) + ', ' + str(self.goal_pose.pose.position.z) + ')')


        while not self.chegou():
            rospy.loginfo('Executing State RTL')
            rospy.loginfo("STARING HOME")
            self.set_position(0,0,height)
            self.rate.sleep()

        t=0
        self.set_position(0,0,height-ds)
        self.rate.sleep()

        init_time = rospy.get_rostime().secs
        #while not (self.drone_pose.pose.position.z < -0.1) and rospy.get_rostime().secs - init_time < (height/velocity)*1.3: #30% tolerance in time
        while not self.LAND_STATE == ExtendedState.LANDED_STATE_ON_GROUND:
            rospy.loginfo('Executing State RTL')

            rospy.loginfo('Height: ' + str(abs(self.drone_pose.pose.position.z)))
            ################# Velocity Control #################
            self.set_vel(0, 0, -velocity, 0, 0, 0)
            rospy.loginfo('LANDING AT ' + str(velocity) + 'm/s')
            self.rate.sleep()
        rospy.logwarn("LANDED_STATE: ON GROUND\nDISARMING")
        self.arm(False)
        return "succeeded"

    def hold(self, time):
        now = rospy.Time.now()
        while not rospy.Time.now() - now > rospy.Duration(secs=time):
            self.local_position_pub.publish(self.drone_pose)
            self.rate.sleep()
    

    def land(self):
        velocity = 0.3
        height = self.drone_pose.pose.position.z
        init_time = rospy.get_rostime().secs
        #while not (self.drone_pose.pose.position.z < 0.4 and not rospy.is_shutdown()) or  self.LAND_STATE == ExtendedState.LANDED_STATE_ON_GROUND:
        while not self.LAND_STATE == ExtendedState.LANDED_STATE_ON_GROUND:
            rospy.logwarn('Landing')
            rospy.loginfo('Height: ' + str(abs(self.drone_pose.pose.position.z)))
            ################# Velocity Control #################
            self.set_vel(0, 0, -velocity, 0, 0, 0)
            self.rate.sleep()
        rospy.logwarn("LANDED_STATE: ON GROUND\nDISARMING")
        self.arm(False)
        return "succeeded"

    def _disarm(self):
        rospy.logwarn("DISARM MAV")
        if drone_pose.pose.position.z < TOL:
            for i in range(3):
                rospy.loginfo('Drone height' + str(drone_pose.pose.position.z))
                self.arm(False)
        else:
            rospy.logwarn("Altitude too high for disarming!")
            self.land()
            self.arm(False)

if __name__ == '__main__':
    mav = MAV("1") #MAV name
    mav.takeoff(3)
    mav.RTL()
