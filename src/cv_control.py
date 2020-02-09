#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist, Vector3Stamped

from dynamic_reconfigure.server import Server
from viscon.cfg import ControllerConfig

from simple_pid import PID
import time
import numpy as np

class VisCon():

    def __init__(self):
        # ROS setup
        rospy.init_node('ObjectBasedController')
        self.rate = rospy.Rate(60)

        # ROS Parameters
        self.vel_topic = rospy.get_param("/vel_topic")
        #self.pose_topic = rospy.get_param("/pose_topic")

        # self.pid_config_file = rospy.get_param("~pid_config_file")
        # self.calibrate_pid = rospy.get_param('~calibrate_pid',False)

        # Publishers
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)

        # Subscribers
        self.detection_sub = rospy.Subscriber('/cv_detection/detection', Vector3Stamped, self.detection_callback)
        self.last_time = time.time()
        self.delay = 0
        # Servers
        self.cfg_srv = Server(ControllerConfig, self.cfg_callback)

        # Attributes
        self.velocity = Twist()
        self.scale_factor = 1
        self.is_losted = True
        # PIDs
        self.pid_x = PID(0.08, 0, 0)    # size
        self.pid_y = PID(0.1, 0, 0)
        self.pid_z = PID(-0.1, 0, 0) # Negative parameters (CV's -y -> Frame's +z)
        self.pid_w = PID(0, 0, 0) #orientation

        self.pid_x.output_limits = self.pid_y.output_limits = (-1, 1) # output value will be between -1 and 1
        self.pid_z.output_limits = (-0.8, 0.8)  # output value will be between -0.8 and 0.8
        
    def set_goal_pose(self, x, y, z, w):
        self.pid_x.setpoint = x
        self.pid_y.setpoint = y
        self.pid_z.setpoint = z # size
        self.pid_w.setpoint = w # orientation

    def set_goal_vel(self, vx, vy, vz, vw):
        self.velocity.linear.x = vx
        self.velocity.linear.y = vy
        self.velocity.linear.z = vz
        self.velocity.angular.z = vw # not volkswagen

    def detection_callback(self, vector_data):
        self.detection = vector_data

        self.velocity.linear.x = self.pid_x(self.detection.vector.z)
        self.velocity.linear.y = self.pid_y(self.detection.vector.x)
        self.velocity.linear.z = self.pid_z(self.detection.vector.y) # PID z must have negative parameters
        self.velocity.angular.z = 0 # TODO implement self.pid_w(orientation)
        
        self.delay = time.time() - self.last_time
        self.is_losted = self.delay > 1
        if not self.is_losted:
            self.vel_pub.publish(self.velocity)

        else:
            rospy.loginfo("Orb slam timeout: {}".format(str(self.delay)))
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

            t = 0
            while self.is_losted:
                self.set_goal_vel(0, 0, 0, 0)
                self.vel_pub.publish(velocity)
            self.rate.sleep()



if __name__ == "__main__":
    c = VisCon()
    c.run()