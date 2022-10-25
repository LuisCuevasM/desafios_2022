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
		self.sub = rospy.Subscriber("/duckiebot/posicionPato", Point, )
        #sucribir a possible_cmd
		self.sub = rospy.Subscriber("/duckiebot/possible_cmd", Twist2DStamped, )
        #publicar la intrucciones en la ruedas
		self.publi = rospy.Publisher("duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = "x")
		self.twist = Twist2DStamped()


        #def intervenir(self, xxx): 
        # if distancia pato < 1:
            #self.twist.omega = 0
			#self.twist.v = 0 
        # else:
            #none

        #def intervenir(self, ):

        #publicar en las ruedas:
        #self.publi.publish(self.twist)

