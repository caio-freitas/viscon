#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy
import rospkg
import ros_numpy
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Empty, Bool
import json, yaml
import sys, select, termios, tty
from rospy_message_converter import json_message_converter

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

s: save position
f: save trajectory file
a: autonomous control ON/OFF


anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

actions = {
    '{':'take_off',
    '}':'land',
    's':'save_pose',
    'f':'save_file',
    'a':'autonomous control ON',
    'z':'autonomous control OFF'}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def pose_callback(data):
    global drone_pose
    drone_pose = data.pose

takeoff_topic = rospy.get_param("/takeoff_topic")
land_topic = rospy.get_param("/land_topic")
vel_topic = rospy.get_param("/vel_topic")
pose_topic = "/orb_slam2_mono/pose"
drone_pose = Pose()

if takeoff_topic != '': # suppose it's mavros -> no takeoff topic
    takeoff_pub = rospy.Publisher(takeoff_topic, Empty, queue_size=1)
    land_pub = rospy.Publisher(land_topic, Empty, queue_size=1)

auto_pub = rospy.Publisher('/control/set_running_state', Bool, queue_size=1)

pose_sub = rospy.Subscriber(pose_topic, PoseStamped, pose_callback)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher(vel_topic, Twist, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    poses = []

    autonomous = False
    rospack = rospkg.RosPack()
    filename = str(rospack.get_path('viscon')+'/config/recorded_routines.json')
    
    with open(filename, 'r') as json_data_file:
        try:
            positions_data = json.load(json_data_file)
        except:
            positions_data = {}

    try:
        print(msg)
        print(vels(speed,turn))
        while not rospy.is_shutdown():
            key = getKey()
            if key in actions.keys():
                if key == '{':
                    takeoff_pub.publish(Empty())
                elif key == '}':
                    land_pub.publish(Empty())
                elif key == 's':
                    print("Saved position: ({}, {}, {})".format(drone_pose.position.x, drone_pose.position.y, drone_pose.position.z))
                    poses.append(drone_pose)
                elif key == 'f':
                    print("Saving file: " + filename)  
                    json_str = []
                    for pose in poses:
                        json_str.append(json_message_converter.convert_ros_message_to_json(pose))

                    with open(filename, 'w') as json_data_file:
                        json.dump(json_str, json_data_file)
                elif key == 'a':
                    autonomous = not autonomous
                    print("Autonomous mode: " + str(autonomous))
                    auto_pub.publish(Bool(autonomous))

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
