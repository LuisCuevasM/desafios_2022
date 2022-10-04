#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist


class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		#suscriber a la camara
		self.Sub_Cam = rospy.Subscriber("/duckiebot/camera_node/image/rec", Image, self.callback)
		#publicar (por definir donde)
		self.pub_img = rospy.Publisher("/duckiebot/camera_node/image/imagen_publi", Image, queue_size = 10)		
		self.detector = cv2.CascadeClasssfier("/home/duckiebot/duckietown/catkin_ws/src/desafios_2022/src/cascade3_LBP_.xml")
	
	#def publicar(self):

	def callback(self,msg):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(msg, "bgr8") 
		img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		
		dets = self.detector.detecMultiScale(img_gray, 1.3, 10)
		
		for (x,y,w,h) in dets:
				
				if w < 100 and h < 100:
					cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 2)
					

		msg = bridge.cv2_to_imgmsg(img_gray, "mono8")
		self.pub_img.publish(msg)
		msg = bridge.cv2_to_imgmsg(image, "bgr8")
		self.pub_img2.publish(msg)

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	#rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
