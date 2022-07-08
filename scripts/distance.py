#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image as ROS_img
from cv_bridge import *
import cv2
import numpy as np
#from PIL import Image
from pyzbar.pyzbar import decode #para detectar los codigos QR
import math

rospy.init_node('distance_node')
image_pub=rospy.Publisher("/distance_estimation",ROS_img, queue_size=1)

def euclidean_distance(edge):        
    #calcula la distancia euclidea entre dos puntos
    pt1, pt2 = edge 
    return math.sqrt(abs((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2))

def max_edge(pts): 
    #calcula la arista mas larga del QR
    pt_top = pts[0][3]
    pt_bottom = pts[0][1]
    pt_left = pts[0][0]
    pt_right = pts[0][2]

    edges_list = [[pt_top, pt_right], [pt_right, pt_bottom], [pt_bottom, pt_left], [pt_left, pt_top]]

    dist1 = euclidean_distance(edges_list[0])
    dist2 = euclidean_distance(edges_list[1])
    dist3 = euclidean_distance(edges_list[2])
    dist4 = euclidean_distance(edges_list[3])

    dist = [dist1, dist2, dist3, dist4]
    dist_max = max(dist)
    max_index = dist.index(dist_max)
    edge_max= edges_list[max_index]

    return dist_max, edge_max

def image_callback(msg):
    frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    for barcode in decode(frame):
        # bounding box
        pts = np.array([barcode.polygon], np.int32)
        cv2.polylines(frame, [pts], True, (0,255,255),2)

        # Punto para el texto
        (x, y, w, h) = barcode.rect
        pt_text_x = int(x + w/6)
        pt_text_y = int(y + h/2)

        # Se verifica que el QR sea numerico
        try:
            data_float = float(barcode.data.decode("utf-8"))
        except:
            cv2.putText(frame, "No valido", (pt_text_x,pt_text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            continue
        
        # correcion del error de impresion en el tamaño del QR:
        QR_size = data_float * (0.88)

        # Estimacion de distancia: 
        # toma en cuenta la arista mas larga del QR
        dist_max, edge_max = max_edge(pts)
        dist_cm = 10601*dist_max**(-0.966)        
        dist_cm = (QR_size/15)*dist_cm

        cv2.putText(frame, str(round(dist_cm,1))+ " cm", (pt_text_x, pt_text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 4)
        cv2.line(frame, edge_max[0], edge_max[1], (0,255, 0), 3)

    #cv2.imshow('distance_node', frame)
    cv2.waitKey(10)
    image_msg = CvBridge().cv2_to_imgmsg(frame,"bgr8")
    image_pub.publish(image_msg)

rospy.Subscriber("/video_stream", ROS_img, image_callback)

try:
    #como no hay un loop, para que el codigo siga corriendo se usa:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()