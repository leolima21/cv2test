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
			ros_frame = self.bridge.cv2_to_imgmsg(cv2_frame, "bgr8")
			self.pub.publish(ros_frame)

# main function
if __name__	== '__main__':
	try:
		campub = camera()
		campub.pub_img()
	except rospy.ROSInterruptException:
		pass			
