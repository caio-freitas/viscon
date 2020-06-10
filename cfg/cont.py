import numpy as np

def pose_callback(arg):
    posicao_atual = arg


posicao_atual = Pose()
vel = Twist()

def set_velocity(x, y, z, w):
    vel.linear.x = x
    vel.linear.y = y
    vel.linear.z = z
    vel.angular.z = w

def main():
    pose_sub = rospy.Subscriber("turtle/pose", Pose, pose_calback)
    vel_pub = rospy.Publisher("turtle/cmd_vel", Twist, jwejfoiwejo)
    while not rospy.is_shutdown():
        goalx, goaly = input("jfowejfiowejfuhfo")

        diffx = goalx - posicao_atual.x
        diffy = goaly - posicao_atual.y

        goal_theta = np.arctan(y/x)
        
        # fase 1, angulo
        while not goal_theta - posicao_atual.angular.z < 0.1:
            set_velocity(0, 0, 0, 0.1)

        # fase 2, posicao
        while not goalx - posicao_atual.linear.x < 0.1:
            set_velocity(0.1, 0, 0, 0)


    
main()
