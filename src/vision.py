#!/usr/bin/env python
# -*- coding: utf-8 -*-
# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import numpy as np
import cv
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError

#from image_converter import ToOpenCV, ToRos 
# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed


class show_image():
	def __init__(self):
		self.subVideo1   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		self.bridge = CvBridge()
		
	def ReceiveImage(self, data):
		try:
			cv_image=self.bridge.imgmsg_to_cv(data, "bgr8")
			imagen=np.asarray(cv_image)    
			
			
		except CvBridgeError, e:
			print e
		
     		conv_hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
 
		#Establecemos el rango de colores que vamos a detectar
		#En este caso de verde oscuro a verde-azulado claro
		verde_bajos = np.array([60,100,100], dtype=np.uint8) #Valores en HSV
		verde_altos = np.array([105, 200, 180], dtype=np.uint8) 

		#Crear una mascara con solo los pixeles dentro del rango de verdes
		#con in range se crea una imagen binarizada
		mascara = cv2.inRange(conv_hsv, verde_bajos, verde_altos)
		edges = cv2.Canny(mascara,100,200)

		#Encontrar el area de los objetos que detecta la camara, con esta funcion
		#se obtienen parámetros como area, equivalente a regionprops en MATLAB
		moments = cv2.moments(mascara)
		area = moments['m00']  #en m00 se guarda el área

		if(area > 200000):  #colocaremos una área mínima
		 
			#Buscamos el centro x, y del objeto
			x = int(moments['m10']/moments['m00']) 
			y = int(moments['m01']/moments['m00'])

			#Dibujamos una marca en el centro del objeto
			cv2.circle(imagen, (x, y), 10,(0,0,255), 2) #sección mejorable
		
		cv2.imshow('Camara', imagen)
		
		
def main(args):
	imagen_mostrar= show_image()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv.DestroyAllWindows() 

if __name__=='__main__':
	main(sys.argv)
