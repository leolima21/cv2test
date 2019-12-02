#!/usr/bin/env python 
import numpy as np 
import cv2

cap = cv2.VideoCapture(0)


while(True):
	# Capture frame-by-frame
	ret, frame = cap.read()
  
	# Display the resulting frame
	cv2.imshow('frame', frame)

	if cv2.waitKey(1) == ord('q'):
		break