#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Pose, PoseStamped
import time

class Head():
    def __init__(self):
        rospy.init_node("head")
        self.rate = rospy.Rate(60)
        self.slam_pose_sub = rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.pose_callback)
        self.cmd_pose_pub = rospy.Publisher('/viscon/set_position', Pose, queue_size=1)
        
        self.current_pose = PoseStamped()
        self.poses = []

    def pose_callback(self, data):
        self.current_pose = data


    def go_to_pose(self, data):
        goal = Pose()
        goal.position.x = data.pose.position.x
        goal.position.y = data.pose.position.y
        goal.position.z = data.pose.position.z
        init_time = time.time()
        while not time.time() - init_time > 20:
            self.cmd_pose_pub.publish(goal)
            self.rate.sleep()
    
    def save_pose(self):
        self.poses.append(self.current_pose)
    

    def run(self):
        s = 's'
        for i in range(3):
            s = input("Save position (s):\n")
            print(s)
            if str(s) == 's':
                self.save_pose()
        for i in range(3):
            self.go_to_pose(self.poses[i])
    

if __name__ == "__main__":
    h = Head()
    h.run()
            