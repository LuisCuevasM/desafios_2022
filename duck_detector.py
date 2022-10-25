#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist, Point # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy



class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		#suscribir a la camara raw 
		self.Sub_Cam = rospy.Subscriber("/duckiebot/camera_node/image/rec", Image, self.procesar_img)
        #publicar en el nodo detecciones
		self.pub_img = rospy.Publisher("/duckiebot/detecciones", Image, queue_size = 1)
        #publicar la posicion del pato
		self.pub_pos = rospy.Publisher("/duckiebot/posicionPato", Point, queue_size = 1)#(x,y,z) #hpato=31 mm 



	#def procesar_img(self):
    #procesa la imagen recibida por la camara -> publica la imagen con la detecciond del pato   
    #y al pocicion del pato

	def procesar_img(self, msg):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(msg, "bgr8") 
		image_out = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
		lower_limit = np.array([60, 100, 100])
		upper_limit =np.array( [100, 255, 255])
		mask = cv2.inRange(image_out, lower_limit, upper_limit)

		img_out = cv2.bitwise_and(image, image, mask=mask)

		# Aplicar mascara
		kernel = np.ones((5,5), np.uint8)
		img_erode = cv2.erode(mask, kernel, iterations=1)
		img_dilate = cv2.dilate(img_erode, kernel, iterations=1)

		# Definir blobs
		distancia = Point()
		_,contours, hierarchy = cv2.findContours(img_dilate,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for cnt in contours:
			AREA = cv2.contourArea(cnt)
			if AREA>300:
				x,y,w,h = cv2.boundingRect(cnt)
				cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 2)
				z = (480/h)
				distancia.x = x
				distancia.y = y
				distancia.z = z
			else:
				None
		# Publicar imagen final
		msg = bridge.cv2_to_imgmsg(image, "bgr8")
		self.pub_img.publish(msg)
		self.pub_pos.publish(distancia)
def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()