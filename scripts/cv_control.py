#!/usr/bin/env python
import rospy
from cv_detection.msg import H_info
from geometry_msgs.msg import TwistStamped
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
        #self.vel_topic = "/mavros/cmd_vel" # Tello
        # self.vel_topic = "/mavros/setpoint_velocity/cmd_vel"
        self.vel_topic = rospy.get_param("/mavros_velocity_pub")
        #self.pose_topic = rospy.get_param("/pose_topic")

        # self.pid_config_file = rospy.get_param("~pid_config_file")
        # self.calibrate_pid = rospy.get_param('~calibrate_pid',False)

        # Publishers
        self.vel_pub = rospy.Publisher(self.vel_topic, TwistStamped, queue_size=1)

        # Subscribers
        self.detection_sub = rospy.Subscriber('/cv_detection/detection', H_info, self.detection_callback)
        
        self.last_time = time.time()
        # Servers
        self.cfg_srv = Server(ControllerConfig, self.cfg_callback)

        # Attributes
        self.delay = 0
        self.velocity = TwistStamped()
        self.scale_factor = 1
        self.is_losted = True
        self.last_time = time.time()
        # PIDs
        self.pid_x = PID(-0.01, -0.008, -0.0001)         # size how close the drone is to the H
        self.pid_y = PID(0.01, 0.008, 0.0001)
        self.pid_z = PID(-0.1, -0.001, 0.0001) # Negative parameters (CV's -y -> Frame's +z)
        self.pid_w = PID(0, 0, 0) # Orientation

        self.pid_x.output_limits = self.pid_y.output_limits = (-0.5, 0.5) # output value will be between -0.3 and 0.3
       
        self.pid_z.output_limits = (-0.5, 0.5)  # output value will be between -0.8 and 0.8
    
    def running_callback(self, bool):
        rospy.logwarn("CV Control activated")
        self.running_state = bool.data


    
    def set_goal_pose(self, x, y, z, w):
        self.pid_x.setpoint = -240.0/2 # y size
        self.pid_y.setpoint = 320.0/2 # x
        self.pid_z.setpoint = 0.01 # 1% of the image area
        self.pid_w.setpoint = 0 # orientation

    def set_goal_vel(self, vx, vy, vz, vw):
        self.velocity.twist.linear.x = vx
        self.velocity.twist.linear.y = vy
        self.velocity.twist.linear.z = vz
        self.velocity.twist.angular.z = vw # not volkswagen

    def detection_callback(self, vector_data):
        self.detection = vector_data
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
            self.delay = time.time() - self.last_time
            if self.running_state:
                self.delay = time.time() - self.last_time
                self.is_losted = self.delay > 5
                if not self.is_losted:
                    self.velocity.twist.linear.x = self.pid_x(-self.detection.center_y)
                    self.velocity.twist.linear.y = self.pid_y(self.detection.center_x)
                    self.velocity.twist.linear.z = self.pid_z(-self.detection.area_ratio) # PID z must have negative parameters
                    self.velocity.twist.angular.z = 0       # TODO implement self.pid_w(orientation)
            
                else:                                       # Assume velocity message will be treated

                    rospy.loginfo("Timeout: {}".format(str(self.delay)))
                    self.velocity.twist.linear.x = 0
                    self.velocity.twist.linear.y = 0
                    self.velocity.twist.linear.z = 0
                    self.velocity.twist.angular.z = 0 # TODO implement self.pid_w(orientation)
                
                self.vel_pub.publish(self.velocity)
                    # Assume velocity message will be treated
                
            self.rate.sleep()

if __name__ == "__main__":
    c = VisCon()
    c.run()