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

# Team ID:		HB_2640
# Author List:		Aryan Gupta
# Filename:		feedback.py
# Functions:	callback(data)
# Nodes:		Publishing node - /controller_node
#				Subscriber node - /gazebo


######################## IMPORT MODULES ##########################

import rospy 			
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation


############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D)
aruco_msg = Pose2D()
pi=math.pi
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
}


##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	rospy.loginfo("receiving camera frame")
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
	arucoParams = cv2.aruco.DetectorParameters_create()
	corners, ids, rejected = cv2.aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParams)
	if len(corners) > 0:
		ids = ids.flatten()
		(topLeft, topRight, bottomRight, bottomLeft) = (corners[0][0][0],corners[0][0][1],corners[0][0][2],corners[0][0][3])
		topRight = (int(topRight[0]), int(topRight[1]))
		bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
		bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
		topLeft = (int(topLeft[0]), int(topLeft[1]))
		bottomcenter=((bottomLeft[0]+bottomRight[0])/2,(bottomLeft[1]+bottomRight[1])/2)
		topcenter=((topLeft[0]+topRight[0])/2,(topLeft[1]+topRight[1])/2)
		hola_theta=math.atan2((topcenter[0]-bottomcenter[0]),(topcenter[1]-bottomcenter[1]))
		if(hola_theta<0):
			hola_theta=pi-abs(hola_theta)
		else:
			hola_theta=-1*(pi-abs(hola_theta))
		
		hola_x=(topcenter[0]+bottomcenter[0])/2
		hola_y=(topcenter[1]+bottomcenter[1])/2
		rospy.loginfo(hola_theta)
		rospy.loginfo(hola_x)
		rospy.loginfo(hola_y)
		aruco_msg.x=hola_x
		aruco_msg.y=hola_y
		aruco_msg.theta=hola_theta
		aruco_publisher.publish(aruco_msg)
			

def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('overhead_cam/image_raw', Image, callback)
	rospy.spin()
  
if __name__ == '__main__':
  main()
