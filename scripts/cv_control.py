#!/usr/bin/env python
import rospy
import math
from cv_detection.msg import H_info
from geometry_msgs.msg import TwistStamped, PoseStamped
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
        self.drone_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        # ROS Parameters
        #self.vel_topic = "/mavros/cmd_vel" # Tello
        # self.vel_topic = "/mavros/setpoint_velocsity/cmd_vel"
        self.vel_topic = rospy.get_param("/mavros_velocity_pub")
        self.pose_topic = rospy.get_param("/mavros_local_atual")
        self.mavros_local_position_pub = rospy.get_param("/mavros_local_position_pub")

        # self.pid_config_file = rospy.get_param("~pid_config_file")
        # self.calibrate_pid = rospy.get_param('~calibrate_pid',False)

        # Publishers
        self.vel_pub = rospy.Publisher(self.vel_topic, TwistStamped, queue_size=1)
        self.cv_control_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)
        self.local_position_pub = rospy.Publisher(self.mavros_local_position_pub, PoseStamped, queue_size = 20)

        # Subscribers
        self.detection_sub = rospy.Subscriber('/cv_detection/detection', H_info, self.detection_callback)
        self.pose = rospy.Subscriber(self.pose_topic, PoseStamped, self.local_callback)
        
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
        #Parametros Proporcional,Integrativo e Derivativo 
        self.pid_x = PID(-0.2, -0.01, -0.1)         
        self.pid_y = PID(0.2, 0.01, 0.1)
        self.pid_z = PID(-0.2, -0.005, -0.001)# Negative parameters (CV's -y -> Frame's +z)
        self.pid_w = PID(0, 0, 0) # Orientation

        #Limitacao da saida        
        self.pid_x.output_limits = self.pid_y.output_limits = (-0.2, 0.2) # output value will be between -0.3 and 0.3
        self.pid_z.output_limits = (-0.15, 0.15)  # output value will be between -0.8 and 0.8

    def local_callback(self, local):
        #Posicao atual do drone
        self.drone_pose.pose.position.x = local.pose.position.x
        self.drone_pose.pose.position.y = local.pose.position.y
        self.drone_pose.pose.position.z = local.pose.position.z
        
    def running_callback(self, bool):
        #Variavel alterada em run_h_mission para iniciar a deteccao do H
        rospy.logwarn("Recebendo info do topicode cv")
        self.running_state = bool.data

    def set_goal_pose(self, x, y, z, w):
        #Posicao que o PID tera como objetivo 
        self.pid_x.setpoint = -240.0/2 # y size
        self.pid_y.setpoint = 320.0/2 # x
        self.pid_z.setpoint = 0.2 # 
        self.pid_w.setpoint = 0 # orientation

    def set_goal_vel(self, vx, vy, vz, vw):
        #Altera as componentes de velocidade do drone
        self.velocity.twist.linear.x = vx
        self.velocity.twist.linear.y = vy
        self.velocity.twist.linear.z = vz
        self.velocity.twist.angular.z = vw # not volkswagen

    def detection_callback(self, vector_data):
        #Dados enviados pelo H.cpp -> Centro do H e Proximidade do H (Area ratio)
        self.detection = vector_data
        self.last_time = time.time()
 
    def set_position(self, x, y, z):
        #Define o objetivo de posicao do drone
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = z
        self.local_position_pub.publish(self.goal_pose)
        self.rate.sleep()

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
        
        self.scale_factor = config.scale_factor

        self.set_goal_pose(config.position_x,
                            config.position_y,
                            config.position_z,
                            config.position_w)

        return config

    def run(self):
        #Inicializacao das variaveis usadas caso o drone perca o H
        r = 0                                        #Inicia valor para o raio da espiral 
        teta = 0                                     #Inicia valor para o teta da espiral 
        last_x = self.drone_pose.pose.position.x    
        last_y = self.drone_pose.pose.position.y
        last_z = self.drone_pose.pose.position.z

        self.set_goal_pose(0, 0, 0, 0) #Config do PID

        while not rospy.is_shutdown():
            self.pose
            self.delay = time.time() - self.last_time
            if self.running_state:  #Condicao alterada no run_h_mission
                self.delay = time.time() - self.last_time
                self.is_losted = self.delay > 5 
                if not self.is_losted:  
                    if self.detection.area_ratio < 0.1: #Drone ainda esta longe do H
                        r = 0
                        teta = 0
                        rospy.loginfo("PID...")
                        self.velocity.twist.linear.x = self.pid_x(-self.detection.center_y)
                        self.velocity.twist.linear.y = self.pid_y(self.detection.center_x)
                        self.velocity.twist.linear.z = self.pid_z(-self.detection.area_ratio) # PID z must have negative parameters
                        self.velocity.twist.angular.z = 0       # TODO implement self.pid_w(orientation) 
                        #Armazena ultima posicao em que o drone nao estava perdido
                        last_x = self.drone_pose.pose.position.x
                        last_y = self.drone_pose.pose.position.y
                        last_z = self.drone_pose.pose.position.z
              
                    else:
                        #Caso o drone esteja suficientemente perto do H
                        rospy.loginfo("Fim controle...")
                        self.cv_control_publisher.publish(Bool(False)) #Volta para o run_h_mission

                    self.vel_pub.publish(self.velocity)

                else:  #Drone perdeu o H
                    #rospy.loginfo("Timeout: {}".format(str(self.delay))
                    ### Fazer espiral ###
                    rospy.loginfo("Fazendo espiral pequena")
                    #Funcao de espiral e volta para dois metros de altura
                    r += 0.002  
                    teta += 0.03
                    x = last_x - r * math.cos( teta ) 
                    y = last_y - r * math.sin( teta )
                    self.set_position(x,y,2)
                    if (r > 1.5):   #limita o tamanho da espiral
                        r = 0

            self.rate.sleep()

if __name__ == "__main__":
    c = VisCon()
    c.run()