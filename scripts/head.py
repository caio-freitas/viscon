#!/usr/bin/env python
import ros_numpy
import numpy as np
import json
import rospy
import rospkg
import select
from std_msgs.msg import ColorRGBA, Bool
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from rospy_message_converter import json_message_converter

import time

class Head():
    def __init__(self):
        # ROS setup
        rospy.init_node("head")
        self.rate = rospy.Rate(60)

        self.running_sub = rospy.Subscriber('/control/set_running_state', Bool, self.running_cb)
        self.slam_pose_sub = rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.pose_callback)
        self.cmd_pose_pub = rospy.Publisher('/viscon/set_position', Pose, queue_size=1)
        self.visual_marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/viscon/path', Path, queue_size=5)
        self.running = False
        self.current_pose = PoseStamped()
        self.goal = Pose()

        self.path = Path()
        self.path.header.frame_id = "map"
            
        self.current_pose_marker = Marker()
        self.current_pose_marker.header.frame_id = "map"
        self.current_pose_marker.ns = "/"
        self.current_pose_marker.id = 0
        self.current_pose_marker.type = Marker.SPHERE
        self.current_pose_marker.action = Marker.MODIFY
        self.current_pose_marker.scale = Vector3(0.1, 0.1, 0.1)
        self.current_pose_marker.color = ColorRGBA(r=0, g=1, b=0, a=0.9)
        self.current_pose_marker.lifetime = rospy.Duration(secs=15)
        self.PRECISION = 0.01 # out of scale, for now

        self.poses = []


    def running_cb(self, data):
        self.running = data.data
        


    def pose_callback(self, data):
        self.current_pose = data


    def is_equal(self, pose, goalpose):
        pose_np = ros_numpy.numpify(pose.pose)
        goalpose_np = ros_numpy.numpify(goalpose)
        return np.linalg.norm(goalpose_np - pose_np) < self.PRECISION

    def go_to_pose(self, data):
        self.goal.position.x = data.position.x
        self.goal.position.y = data.position.y
        self.goal.position.z = data.position.z

        self.current_pose_marker.pose = self.goal
        self.current_pose_marker.id += 1
        self.current_pose_marker.header.stamp = rospy.Time()
        
        init_time = time.time()
        while not time.time() - init_time > 10 and not rospy.is_shutdown() and not self.is_equal(self.current_pose, self.goal):
            self.visual_marker_pub.publish(self.current_pose_marker)
            rospy.logwarn("Going to position: "+ str(self.goal.position.x) + ", " + str(self.goal.position.y) + ", " + str(self.goal.position.z))
            rospy.logwarn("Currently at: "+ str(self.current_pose.pose.position.x) + ", " + str(self.current_pose.pose.position.y) + ", " + str(self.current_pose.pose.position.z))
            self.cmd_pose_pub.publish(self.goal)    #rviz show next pose
            self.path_pub.publish(self.path)        #rviz show path
            self.rate.sleep()
            if not self.running:
                break

    def save_pose(self):
        self.poses.append(self.current_pose)
    

    def run(self):
        rospack = rospkg.RosPack()
        filename = str(rospack.get_path('viscon')+'/config/recorded_routines.json')
        
        with open(filename, 'r') as json_data_file:
            try:
                self.positions_data = json.load(json_data_file)
            except Exception as e:
                print(e)
                self.positions_data = {}
        for i in range(len(self.positions_data)):
            json_to_pose = json_message_converter.convert_json_to_ros_message('geometry_msgs/Pose', self.positions_data[i])
            json_to_pose_stamped = PoseStamped()
            json_to_pose_stamped.pose = json_to_pose
            self.path.poses.append(json_to_pose_stamped)
        # if self.running:
        #     for i in range(len(positions_data)):
        #         self.go_to_pose(positions_data[str(i)])
        #         if not self.running:
        #             break
        while not rospy.is_shutdown():
            if self.running:
                rospy.loginfo("Running...")
                for i in range(len(self.positions_data)):
                    json_to_pose = json_message_converter.convert_json_to_ros_message('geometry_msgs/Pose', self.positions_data[i])
                    self.go_to_pose(json_to_pose)
                    if not self.running:
                        break
            self.rate.sleep()
        

def main():
    h = Head()

if __name__ == "__main__":
    # main()
    h = Head()
    h.run()
            