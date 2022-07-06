#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image as rosim
from cv_bridge import *
import cv2
import numpy as np
from PIL import Image
from pyzbar.pyzbar import decode #para detectar los codigos QR
import math

rospy.init_node('distance_node')
image_pub=rospy.Publisher("/distance_estimation",rosim, queue_size=1)
QR_size = 17 #cm

def euclidean_distance(pt1, pt2):
    return math.sqrt(abs((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2))

def image_callback(msg):
    frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    for barcode in decode(frame):
        
        data_string = barcode.data.decode("utf-8")
        print(data_string)

        pts = np.array([barcode.polygon], np.int32)

        cv2.polylines(frame, [pts], True, (0,255,255),3) # bounding box

        #distancia version 1: toma en cuenta la diagonal que parte QR a la mitad
        #dist_pixels = euclidean_distance(pt_top, pt_bottom)
        #dist_cm = 29260*dist_pixels**(-1.08)

        #distancia version 2: toma en cuenta la arista mas larga del QR

        pt_top = pts[0][3]
        pt_bottom = pts[0][1]
        pt_left = pts[0][0]
        pt_right = pts[0][2]

        dist1 = euclidean_distance(pt_top, pt_right)
        dist2 = euclidean_distance(pt_right, pt_bottom)
        dist3 = euclidean_distance(pt_bottom, pt_left)
        dist4 = euclidean_distance(pt_left, pt_top)
        dist = [dist1, dist2, dist3, dist4]
        dist_max = max(dist)

        dist_cm = 17961*dist_max**(-1.05)
        dist_cm = (QR_size/17)*dist_cm

        cv2.putText(frame, str(round(dist_cm,2))+ " cm", (460, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    #cv2.imshow('distance_node', frame)
    cv2.waitKey(10)
    image_msg = CvBridge().cv2_to_imgmsg(frame,"bgr8")
    image_pub.publish(image_msg)		

rospy.Subscriber("/video_stream", rosim, image_callback)

try:
    #como no hay un loop, para que el codigo siga corriendo se usa:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()