#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import *
import cv2
import numpy as np

rospy.init_node('image_msg_suscriber')

def image_callback(msg):   
    #ROS Image to OpenCV2
    img = CvBridge().imgmsg_to_cv2(msg, "bgr8")    

    #Redimensionamiento
    scale_percent = 100
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)    

    #mostrar imagen
    cv2.imshow("image_msg_suscriber", img)

    #guardar imagen
    #cv2.imwrite('camera_image_{}.png'.format(a), dst)
    
    cv2.waitKey(1)    

#Topico de la camara: /video_stream
#Topico del resiltado: /distance_estimation
rospy.Subscriber("/distance_estimation", Image, image_callback)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()