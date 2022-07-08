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


Para los ejemplos se uso un codigo QR de 17 x 17 cm

---
Si quiere probarlo con su propia cámara corra:

```
roslaunch distance_meter camera.launch
```

> No olvide cambiar el dispositivo en ***image_msg_publisher.py***, linea 10:
```{python3}
cap=cv2.VideoCapture('/dev/video4')
```
El codigo QR debe contener el dato de su longitud en centimetros. Puede revisar un ejemplo en el archivo ***qr.pdf***