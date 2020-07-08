#!/usr/bin/env python
import rospy
from cv_detection.msg import H_info
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from dynamic_reconfigure.server import Server
from viscon.cfg import ControllerConfig

from simple_pid import PID

import time
import numpy as np

class VisCon():


    def __init__(self):
        # ROS setup
        rospy.init_node('control')
        self.rate = rospy.Rate(60)

        # State updater
        self.running_state = False
        self.running_sub = rospy.Subscriber("/cv_detection/set_running_state", Bool, self.running_callback)

        # ROS Parameters
        self.vel_topic = "/tello/cmd_vel" # Tello
        # self.vel_topic = "/mavros/setpoint_velocity/cmd_vel"
        #self.vel_topic = rospy.get_param("/vel_topic")
        #self.pose_topic = rospy.get_param("/pose_topic")

        # self.pid_config_file = rospy.get_param("~pid_config_file")
        # self.calibrate_pid = rospy.get_param('~calibrate_pid',False)

        # Publishers
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)

        # Subscribers
        self.detection_sub = rospy.Subscriber('/cv_detection/detection', H_info, self.detection_callback)
                                                #'/viscon/cv_control/set_running_state'
        self.last_time = time.time()
        rospy.init_node('control')
        self.rate = rospy.Rate(60)

        # Servers
        self.cfg_srv = Server(ControllerConfig, self.cfg_callback)

        # Attributes
        self.delay = 0
        self.velocity = Twist()
        self.scale_factor = 1
        self.is_losted = True
        self.last_time = time.time()
        # PIDs
        self.pid_x = PID(0.00001, 0, 0)         # size how close the drone is to the H
        self.pid_y = PID(0.00081, 0.00001, 0)
        self.pid_z = PID(-0.00081, -0.00001, 0) # Negative parameters (CV's -y -> Frame's +z)
        self.pid_w = PID(0, 0, 0) # Orientation

        self.pid_x.output_limits = self.pid_y.output_limits = (-0.3, 0.3) # output value will be between -0.3 and 0.3
       
        self.pid_z.output_limits = (-0.3, 0.3)  # output value will be between -0.8 and 0.8
    
    def running_callback(self, bool):
        self.running_state = bool.data


    
    def set_goal_pose(self, x, y, z, w):
        self.pid_x.setpoint = 0.01 # 1% of the image area
        self.pid_y.setpoint = 960.0/2 #x
        self.pid_z.setpoint = -720.0/2 # y size
        self.pid_w.setpoint = 0 # orientation

    def set_goal_vel(self, vx, vy, vz, vw):
        self.velocity.linear.x = vx
        self.velocity.linear.y = vy
        self.velocity.linear.z = vz
        self.velocity.angular.z = vw # not volkswagen

    def detection_callback(self, vector_data):
        self.detection = vector_data
        
        self.delay = time.time() - self.last_time
        self.is_losted = self.delay > 1
        if not self.is_losted:
            self.velocity.linear.x = self.pid_x(self.detection.area_ratio)
            self.velocity.linear.y = self.pid_y(self.detection.center_x)
            self.velocity.linear.z = self.pid_z(-self.detection.center_y) # PID z must have negative parameters
            self.velocity.angular.z = 0 # TODO implement self.pid_w(orientation)
        
            self.vel_pub.publish(self.velocity)

        else:            # Assume velocity message will be treated

            rospy.loginfo("Timeout: {}".format(str(self.delay)))
            self.velocity.linear.x = 0
            self.velocity.linear.y = 0
            self.velocity.linear.z = 0
            self.velocity.angular.z = 0 # TODO implement self.pid_w(orientation)
        
            self.vel_pub.publish(self.velocity)
            # Assume velocity message will be treated
        self.last_time = time.time()
        rospy.loginfo(self.velocity) # debug

    def cfg_callback(self, config, level):
        
        if hasattr(self, 'pid_x'):
            self.pid_x.tunings = (config.p_x, config.i_x, config.d_x)
            self.pid_y.tunings = (config.p_y, config.i_y, config.d_y)
            self.pid_z.tunings = (config.p_z, config.i_z, config.d_z)
            self.pid_w.tunings = (config.p_w, config.i_w, config.d_w)
        else:
            self.pid_x = PID(config.p_x, config.i_x, config.d_x)
            self.pid_y = PID(config.p_y, config.i_y, config.d_y)
            self.pid_z = PID(config.p_z, config.i_z, config.d_z)
            self.pid_w = PID(config.p_w, config.i_w, config.d_w)
        #print(config)
        self.scale_factor = config.scale_factor

        self.set_goal_pose(config.position_x,
                            config.position_y,
                            config.position_z,
                            config.position_w)

        return config

    def run(self):
        self.set_goal_pose(0, 0, 0, 0)
        while not rospy.is_shutdown():
            if self.running_state:
                t = 0
                while self.is_losted:
                    self.set_goal_vel(0, 0, 0, 0)
                    self.vel_pub.publish(self.velocity)
                self.rate.sleep()

if __name__ == "__main__":
    c = VisCon()
    c.run()