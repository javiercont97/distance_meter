#!/usr/bin/env python3
import rospy
import roslaunch
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import linecache

rospy.init_node('image_msg_publisher',anonymous=True)
image_pub=rospy.Publisher("/usb_cam/image_raw",Image, queue_size=1)	
cap=cv2.VideoCapture('/dev/video4')
bridge=CvBridge()


if cap.isOpened():
	while not rospy.is_shutdown():
		try:	
			ret,frame=cap.read()
			#cv2.imshow('image_msg_publisher',frame)			
			#print(frame.shape)

			image_msg = bridge.cv2_to_imgmsg(frame,"bgr8")
			image_pub.publish(image_msg)	
			cv2.waitKey(10)	
				
		except KeyboardInterrupt:
			print("shutting down")
			cv2.destroyAllWindows()
else:
	particular_line = linecache.getline('image_msg_publisher.py', 11)
	print("\nCamara no Disponible, image_msg_publisher.py, linea 11:\n \n	"+particular_line)
	print("puede ver sus camaras disponibles en la terminal con:" +"\n \n"+"	v4l2-ctl --list-devices"+"\n")	