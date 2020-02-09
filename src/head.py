#!/usr/bin/env python

import rospy
import sys, tty
import termios
import select
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from visualization_msgs.msg import Marker

import time

class Head():
    def __init__(self):
        # ROS setup
        rospy.init_node("head")
        self.rate = rospy.Rate(60)

        self.slam_pose_sub = rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.pose_callback)
        self.cmd_pose_pub = rospy.Publisher('/viscon/set_position', Pose, queue_size=1)
        self.visual_marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        
        self.current_pose = PoseStamped()

        self.current_pose_marker = Marker()
        self.current_pose_marker.header.frame_id = "map"
        self.current_pose_marker.ns = "/"
        self.current_pose_marker.id = 0
        self.current_pose_marker.type = Marker.SPHERE
        self.current_pose_marker.action = Marker.MODIFY
        self.current_pose_marker.scale = Vector3(0.2, 0.2, 0.2)
        self.current_pose_marker.color = ColorRGBA(r=0, g=1, b=0, a=0.9)
        self.current_pose_marker.lifetime = rospy.Duration(secs=15)

        self.settings = termios.tcgetattr(sys.stdin)
        self.poses = []

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def pose_callback(self, data):
        self.current_pose = data


    def go_to_pose(self, data):
        goal = Pose()
        goal.position.x = data.pose.position.x
        goal.position.y = data.pose.position.y
        goal.position.z = data.pose.position.z

        self.current_pose_marker.pose = goal
        self.current_pose_marker.id += 1
        self.current_pose_marker.header.stamp = rospy.Time()
        self.visual_marker_pub.publish(self.current_pose_marker)

        init_time = time.time()
        while not time.time() - init_time > 20 and not rospy.is_shutdown():
            rospy.logwarn("Going to position: "+ str(data.pose.position.x) + ", " + str(data.pose.position.y) + ", " + str(data.pose.position.z))
            rospy.logwarn("Currently at: "+ str(self.current_pose.pose.position.x) + ", " + str(self.current_pose.pose.position.y) + ", " + str(self.current_pose.pose.position.z))
            self.cmd_pose_pub.publish(goal)
            self.rate.sleep()


    def save_pose(self):
        self.poses.append(self.current_pose)
    

    def run(self):
        s = 's'
        for i in range(3):
            s = input("Save position (s):\n")
            pos = Pose()
            pos.position.x = self.current_pose.pose.position.x
            pos.position.y = self.current_pose.pose.position.y
            pos.position.z = self.current_pose.pose.position.z
            
            print(s)
            if str(s) == 's':
                self.save_pose()
        for i in range(3):

            self.go_to_pose(self.poses[i])
    

def main():
    h = Head()
    while not rospy.is_shutdown():
        key = h.getKey()
        print(key)
        if key=='q' or ord(key) == 32:
            break
        if ord(key) == 32: # spacebar
            break
if __name__ == "__main__":
    # main()
    h = Head()
    h.run()
            