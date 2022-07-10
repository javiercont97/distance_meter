#!/usr/bin/env python3
import cv2
import numpy as np
from pyzbar.pyzbar import decode #para detectar los codigos QR
import math

cap = cv2.VideoCapture(0)
# FUNCIONES DISTANCE METER
def euclidean_distance(edge):
    #calcula la distancia euclidea entre dos puntos
    pt1, pt2 = edge 
    return math.sqrt(abs((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2))

def euclidean_distance_3D(qr1, qr2):
    #calcula la distancia euclidea entre dos puntos
    return (math.sqrt(abs((qr1[0] - qr2[0])**2 + (qr1[1] - qr2[1])**2 + (qr1[2] - qr2[2])**2)),[(qr1[3], qr1[4]), (qr2[3], qr2[4])])

def max_edge(pts): 
    #calcula la arista mas larga del QR
    pt_top = pts[0][3]
    pt_bottom = pts[0][1]
    pt_left = pts[0][0]
    pt_right = pts[0][2]

    edges_list = [[pt_top, pt_right], [pt_right, pt_bottom], [pt_bottom, pt_left], [pt_left, pt_top]]

    dist = []
    dist.append( euclidean_distance(edges_list[0]) )
    dist.append( euclidean_distance(edges_list[1]) )
    dist.append( euclidean_distance(edges_list[2]) )
    dist.append( euclidean_distance(edges_list[3]) )

    dist_max = max(dist)
    max_index = dist.index(dist_max)
    edge_max= edges_list[max_index]

    return dist_max, edge_max

# Defining the dimensions of checkerboard

while(True):
    ret, dst = cap.read()
    # Refining the camera matrix using parameters obtained by calibration
    dists_barcode = []
    for barcode in decode(dst):
        # bounding box
        pts = np.array([barcode.polygon], np.int32)
        cv2.polylines(dst, [pts], True, (0,255,255),2)

        # Punto para el texto
        (x, y, w, h) = barcode.rect
        pt_text = ( int(x + w/6), int(y + h/2) )
        pt_center = ( int(x + w/2), int(y + h/2) )

        # Se verifica que el QR sea numerico
        try:
            data_float = float(barcode.data.decode("utf-8"))
        except:
            data_float = barcode.data.decode("utf-8")
            cv2.putText(dst, data_float, (30,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            continue
        
        # correcion del error de impresion en el tamaño del QR:
        QR_size = data_float #* (0.88)

        # Estimacion de distancia: 
        # toma en cuenta la arista mas larga del QR
        dist_max, edge_max = max_edge(pts)

        # Para la toma de datos
        #cv2.putText(dst, str(round(dist_max)), (30,100), cv2.FONT_HERSHEY_SIMPLEX, 3, (0,0,255), 8)        

        # Estimacion de distancia: 
        # toma en cuenta la arista mas larga del QR
        dist_max, edge_max = max_edge(pts)
        dist_z = 10601*dist_max**(-0.966)        
        dist_z = (QR_size/15)*dist_z        

        # Se calcula la distancia X en cm
        dist_px_X = pt_center[0] - dst.shape[1]/2
        dist_x = QR_size  * (dist_px_X / w)

        # Se calcula la distancia Y en cm
        dist_px_Y = pt_center[1] - dst.shape[0]/2
        dist_y = QR_size  * (dist_px_Y / h)

        dists_barcode.append( [dist_x, dist_y, dist_z, int(pt_center[0]), int(pt_center[1]) ] )
        #cv2.putText(dst, f"({round(dist_z)}) cm", pt_text, cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 4)
        #cv2.line(dst, (int(dst.shape[1]/2), int(dst.shape[0]/2)), pt_center, (150,150, 50), 2)
        #cv2.line(dst, edge_max[0], edge_max[1], (0,255, 0), 3)



    if len(dists_barcode) > 2:
        dists_barcode.append(dists_barcode[0])

    for i in range(len(dists_barcode)-1):
        #print("hola")
        qr1 = dists_barcode[i]
        qr2 = dists_barcode[i + 1]
        d, line = euclidean_distance_3D(qr1, qr2)

        middle_line = (int((line[0][0] + line[1][0])/2), int((line[0][1] + line[1][1])/2))
        
        cv2.putText(dst, f"({round(d)}) cm", (middle_line), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 4)
        cv2.line(dst, line[0], line[1], (255, 20, 255), 3)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cv2.imshow('distance_node', dst)

# Displaying the undistorted image
cap.release()
cv2.destroyAllWindows()