#!/usr/bin/env python 

#libs:
import rospy
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

class camera:
	def __init__(self):
		# create a node
		rospy.init_node('nodecv2', anonymous=True)
		# publisher object
		self.pub = rospy.Publisher('topiccv2', Image, queue_size=10)
		
		# bridge object
		self.bridge = CvBridge()

		# camera setup
		self.cap = cv2.VideoCapture(0)

	# pub metod
	def pub_img(self):
		while True:
			ret, cv2_frame = self.cap.read()

			cv2_frame = cv2.medianBlur(cv2_frame, 5)
			# cimg = cv2.cvtColor(cv2_frame, cv2.COLOR_GRAY2BGR)
			gray = cv2.cvtColor(cv2_frame, cv2.COLOR_BGR2GRAY)


			circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,1000, param1=30,param2=50,minRadius=20,maxRadius=150)

			circles = np.uint16(np.around(circles))
			
			for i in circles[0,:]:
				cv2.circle(cv2_frame,(i[0],i[1]), i[2], (0,255,0),2)
				cv2.circle(cv2_frame,(i[0],i[1]), 2, (0,0,255),3)
			cv2.imshow('detected circles', cv2_frame)
			# cv2.namedWindow('r', cv2.WINDOW_NORMAL)
			# cv2.namedWindow('g', cv2.WINDOW_NORMAL)
			# cv2.namedWindow('b', cv2.WINDOW_NORMAL)
			# cv2.namedWindow('x', cv2.WINDOW_NORMAL)
			
			# # rgb var
			# r = cv2_frame[:, :, 2]
			# g = cv2_frame[:, :, 1]
			# b = cv2_frame[:, :, 0]

			# # effects
			# b[(b > 100) & (g < 200)] = 255
			# b[b < 255] = 0

			# # merge rgb channels
			# x = cv2_frame
			# x[:,:,0] = b
			# x[:,:,1] = g
			# x[:,:,2] = r
			
			# # show rgb channels
			# cv2.imshow('b', b)
			# cv2.imshow('g', g)
			# cv2.imshow('r', r)
			# cv2.imshow('x', x)
			
			if cv2.waitKey(1) == ord('q'):
				break

			ros_frame = self.bridge.cv2_to_imgmsg(cv2_frame, "bgr8")
			self.pub.publish(ros_frame)


# main function
if __name__	== '__main__':
	try:
		campub = camera()
		campub.pub_img()
	except rospy.ROSInterruptException:
		pass			
