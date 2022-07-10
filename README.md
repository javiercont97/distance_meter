# Distance Meter

Este repositorio es un package de ROS, por lo que se puede clonar en el src de un workspace. El proyecto se desarrollo en ROS Noetic y Ubuntu 20.04.

Para correr los códigos con los archivos .launch que se indican a continuación, recuerde hacer ejecutables los archivos .py en la ***carpeta scripts***. Puede usar el siguiente comando:
```
chmod +x *.py
```

Es posible que necesite instalarse la libreria pyzbar, el comando para ubuntu es el siguiente:

```
sudo apt-get install libzbar0
```

---
Puede correr el demo con:
```
roslaunch distance_meter demo.launch
```
---
Si quiere probarlo con su propia cámara primero deberá calibrarla. En dos terminales diferentes corra:

```
roslaunch distance_meter camera.launch
```

> No olvide cambiar el dispositivo en ***image_msg_publisher.py***, linea 10:
```{python3}
cap=cv2.VideoCapture('/dev/video4')
```
El codigo QR debe contener el dato de su longitud en centimetros. Puede revisar un ejemplo en el archivo ***qr.pdf***

> En caso de que la medicion de distancia sea incorrecta, es posible que necesite calibrar su camara. Para ello corra en dos terminales diferentes:

1. Para lanzar la camara:
```
roslaunch distance_meter calibration.launch
```
2. Para lanzar el Wizard de calibracion:
```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0235 image:=/usb_cam/image_raw camera:=/usb_cam
```
En este ultimo necesitara un checkerboard, puede utilizar el adjunto en el repositorio.

Aclarar que:
*  **--size** es el numero de intersecciones entre cuadrados negros
*  **--square** es el tamaño de cada cuadrado negro (en metros).

Debera mover el checkerboard frente a la camara hasta que todos los parametros sean de color verde y se habilite el boton de calibrar. 

Una vez calibrado y comiteado, deberia medir correctamente la distancia de los codigos QR.


