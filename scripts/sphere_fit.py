#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

xyz = XYZarray()
sphere_params = SphereParams()
computed = False

def solver(data):

	global computed
	#global points
	#global sphere_params
	
	points = data.points
	
	# develop A and B matrices
	A = []
	B = []
	
	# iterate over points and add to matrices for xyz
	for i in range (len(points)):
		A2 = []
		A2.append(2*points[i].x)
		A2.append(2*points[i].y)
		A2.append(2*points[i].z)
		A2.append(1)
		A.append(A2)
		B.append(points[i].x**2 + points[i].y**2 + points[i].z**2)
	
	# calculate P
	P = np.linalg.lstsq(A,B,rcond = None)[0]
	
	# calculate radius
	r = np.sqrt(P[0]**2 + P[1]**2 + P[2]**2 + P[3])
	
	# assign values to sphere data 
	sphere_params.xc = P[0]
	sphere_params.yc = P[1]
	sphere_params.zc = P[2]
	sphere_params.radius = r
	
	# set flag to True
	computed = True

if __name__ == '__main__':
	# define node
	rospy.init_node('sphere_fit', anonymous = True)
	# define a subscriber to read images
	img_sub = rospy.Subscriber('/xyz_cropped_ball', XYZarray, solver) 
	# define a publisher to publish images
	img_pub = rospy.Publisher('sphere_params', SphereParams, queue_size = 1)
	
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# publish if data has been calculated 
		if computed:
			img_pub.publish(sphere_params)
			computed = False 
		#pause until the next iteration			
		rate.sleep()
