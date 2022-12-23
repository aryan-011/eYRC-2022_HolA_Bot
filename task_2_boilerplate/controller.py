#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		HB2640
# Author List:	Aryan Gupta
# Filename:		feedback.py
# Functions: task2_goals_Cb,aruco_feedback_Cb,inverse_kinematics
# Nodes:	Publishing node - /gazebo
#           Subscribing node - /aruco_feedback_node    
################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user
import numpy as np
from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14

x_goals = []
y_goals = []
theta_goals = []
hola_x=0
hola_y=0
hola_theta=0
right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None




def signal_handler(sig, frame):
	  
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup():
    clean=0
  
  
def task2_goals_Cb(msg):
	global x_goals, y_goals, theta_goals
	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

def aruco_feedback_Cb(msg):
    global hola_theta,hola_x,hola_y
    hola_y=msg.y
    hola_x=msg.x
    hola_theta = msg.theta


def inverse_kinematics(a):
    b=[[-0.15,1,0],[-0.15,-0.5,-math.sin(PI/3)],[-0.15,-0.5,math.sin(PI/3)]]

    c=np.matmul(b,a)
    

    return c



def main():

    rospy.init_node('controller_node')

    signal.signal(signal.SIGINT, signal_handler)

    right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
    front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
    left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)


    rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
    rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)
    rate = rospy.Rate(100)
    frontwheel=Wrench()
    rightwheel=Wrench()
    leftwheel=Wrench()
	   
    global x_goals,y_goals,theta_goals	
    while not rospy.is_shutdown():
        k=-1
        j=0
        for i in range (0,len(x_goals)):
            j+=1
            rospy.loginfo("____________________________")
            rospy.loginfo("-------------------------------")
            x_d=x_goals[i]
            y_d=y_goals[i]
            theta_d=theta_goals[i]

           
            while not rospy.is_shutdown():
                global hola_x,hola_theta,hola_y 

                inc_theta=theta_d-hola_theta
                inc_x=(x_d-hola_x)*0.0002645833
                inc_y=-(y_d-hola_y)*0.0002645833
                kx=50
                ky=50
                ktheta=150
                
                frontwheel.force.y=0
                frontwheel.force.z=0
                rightwheel.force.y=0
                rightwheel.force.z=0
                leftwheel.force.y=0
                leftwheel.force.z=0
                if((math.sqrt(inc_x*inc_x+inc_y*inc_y))<0.001 and abs(inc_theta)<0.0175): 
                    vx=0
                    vy=0
                    w=0
                    t1=rospy.Time.now().to_sec()
                    t2=0
                    rospy.loginfo("goal reached")
                    while(t2-t1<2):
                        frontwheel.force.x=0
                        rightwheel.force.x=0
                        leftwheel.force.x=0
                        front_wheel_pub.publish(frontwheel)
                        right_wheel_pub.publish(rightwheel)
                        left_wheel_pub.publish(leftwheel)
                        t2=rospy.Time.now().to_sec()
                    break
                else:
                    vx=min(max(kx*inc_x*math.cos(hola_theta)+ky*inc_y*math.sin(hola_theta),-3),3)
                    vy=min(max(ky*inc_y*math.cos(hola_theta)-kx*inc_x*math.sin(hola_theta),-3),3)
                    w=min(max(ktheta*inc_theta,-3),3)
                
                if(k<len(x_goals)):
                    k=len(x_goals)

                a=[[w],[vx],[vy]]
                c=inverse_kinematics(a)  
                frontwheel.force.x=c[0][0]*100
                rightwheel.force.x=c[1][0]*100
                leftwheel.force.x=c[2][0]*100
                front_wheel_pub.publish(frontwheel)
                right_wheel_pub.publish(rightwheel)
                left_wheel_pub.publish(leftwheel)             
                rate.sleep()
        rospy.loginfo(k)
        if(k==j):
            break
    while not rospy.is_shutdown():
        continue
		

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

