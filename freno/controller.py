#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 #importa mensajes de ROS tipo String y Int32
from geometry_msgs.msg import Twist, Point # importar mensajes de ROS tipo geometry / Twist y Point
from sensor_msgs.msg import Joy # impor mensaje tipo Joy
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from duckietown_msgs.msg import Twist2DStamped 



class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
        #sucribir a posicionPato
		self.subpos = rospy.Subscriber("/duckiebot/posicionPato", Point, self.intervenir)
        #sucribir a possible_cmd
		self.subjoy = rospy.Subscriber("/duckiebot/possible_cmd", Twist2DStamped, self.callback)
        #publicar la intrucciones en la ruedas
		self.publiruedas = rospy.Publisher("duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = 0)
		self.twist = Twist2DStamped()


        def intervenir(self, msg): 
        	if msg.z < 10:
            		self.twist.omega = 0
			self.twist.v = 0
		self.publiruedas.publish(self.twist)
	
	def callback(self, msg):
		self.twist = msg 
	
        #publicar en las ruedas:
        	
        	
def main():
	rospy.init_node('test1') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
	
