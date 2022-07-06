#!/usr/bin/env python3
import sys
import rospy 
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('image_msg_publisher',anonymous=True)
image_pub=rospy.Publisher("/video_stream",Image, queue_size=1)	
cap=cv2.VideoCapture('/dev/video4')
bridge=CvBridge()
while not rospy.is_shutdown():
	try:
		ret,frame=cap.read()
		#cv2.imshow('image_msg_publisher',frame)
		cv2.waitKey(10)
		image_msg = bridge.cv2_to_imgmsg(frame,"bgr8")
		image_pub.publish(image_msg)
			
	except KeyboardInterrupt:
		print("shutting down")
		cv2.destroyAllWindows()
