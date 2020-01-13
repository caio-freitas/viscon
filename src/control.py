#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist, Pose

class controller():

    self.goal_pose = Pose()
    self.current_pose = Pose()
    self.velocity = Twist()

    self.vel_topic = rospy.get_param("~vel_topic")
    self.pose_topic = rospy.get_param("~pose_topic")

    self.pid_config_file = rospy.get_param("~pid_config_file")

    self.vel_pub = rospy.Publisher(self.vel_topic, Twist)
    self.pose_sub = rospy.Subscribe('/control/position', Pose, self.pose_callback)
    self.relative_pose_sub = rospy.Subscribe('/control/relative_position', Pose, self.pose_callback)
    


    def pose_callback(self, pose_obj):
        self.current_pose = pose_obj

    