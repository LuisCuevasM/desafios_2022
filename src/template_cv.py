#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self.procesar_img)
		self.pub_img = rospy.Publisher("/duckiebot/camera_node/image/cooked", Image, queue_size = 10)

	#def publicar(self):

	def procesar_img(self, msg):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(msg, "bgr8") 
		image_out = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
		msg = bridge.cv2_to_imgmsg(image_out, "bgr8")
		self.pub_img.publish(msg)
		



	

		#def procesar_img(self, img):
		# Cambiar espacio de color

		# Filtrar rango util

		# Aplicar mascara

		# Aplicar transformaciones morfologicas

		# Definir blobs

		# Dibujar rectangulos de cada blob

		# Publicar imagen final


def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
