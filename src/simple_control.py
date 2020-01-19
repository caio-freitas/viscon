#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist, Pose, PoseStamped

from dynamic_reconfigure.server import Server
from viscon.cfg import ControllerConfig

from simple_pid import PID

class VisCon():

    def __init__(self):
        # ROS setup
        rospy.init_node('VisualOdometryController')
        self.rate = rospy.Rate(60)

        # ROS Parameters
        self.vel_topic = rospy.get_param("/vel_topic")
        #self.pose_topic = rospy.get_param("/pose_topic")

        # self.pid_config_file = rospy.get_param("~pid_config_file")
        # self.calibrate_pid = rospy.get_param('~calibrate_pid',False)

        # Publishers
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)

        # Subscribers
        self.pose_sub = rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.pose_callback)
       
        # Servers
        self.cfg_srv = Server(ControllerConfig, self.cfg_callback)

        # Attributes    
        self.goal_pose = PoseStamped()
        self.current_pose = PoseStamped()
        self.velocity = Twist()
        self.scale_factor = 1

        # PIDs
        self.pid_x = PID(1, 0, 0)
        self.pid_y = PID(2, 0, 0)
        self.pid_z = PID(0.5, 0, 0)

        self.pid_x.output_limits = self.pid_y.output_limits = (-1, 1) # output value will be between -1 and 1
        self.pid_z.output_limits = (-0.8, 0.8)  # output value will be between -0.8 and 0.8

    def set_goal_pose(self, x, y, z):
        self.pid_x.setpoint = x
        self.pid_y.setpoint = y
        self.pid_z.setpoint = z

    def pose_callback(self, pose_obj):
        self.current_pose = pose_obj

        self.current_pose.pose.position.x = self.scale_factor * self.current_pose.pose.position.x
        self.current_pose.pose.position.x = self.scale_factor * self.current_pose.pose.position.y
        self.current_pose.pose.position.x = self.scale_factor * self.current_pose.pose.position.z
    
        self.velocity.linear.x = self.pid_x(self.current_pose.pose.position.x)
        self.velocity.linear.y = self.pid_y(self.current_pose.pose.position.y)
        self.velocity.linear.z = self.pid_z(self.current_pose.pose.position.z)
        
        self.vel_pub.publish(self.velocity)

        rospy.loginfo(self.velocity)

    def cfg_callback(self, config, level):
        
        if hasattr(self, 'pid_x'):
            self.pid_x.tunings = (config.p_x, config.i_x, config.d_x)
            self.pid_y.tunings = (config.p_y, config.i_y, config.d_y)
            self.pid_z.tunings = (config.p_z, config.i_z, config.d_z)
        else:
            self.pid_x = PID(config.p_x, config.i_x, config.d_x)
            self.pid_y = PID(config.p_y, config.i_y, config.d_y)
            self.pid_z = PID(config.p_z, config.i_z, config.d_z)
        #print(config)
        self.scale_factor = config.scale_factor
        return config

    def run(self):

        while not rospy.is_shutdown():
            self.set_goal_pose(0, 0, 2)
            rospy.spin()
            #rospy.loginfo(self.velocity)


if __name__ == "__main__":
    c = VisCon()
    c.run()
