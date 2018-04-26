#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped

class controller(object):
	def __init__(self):
		super(controller, self).__init__() 
		self.subscriber=rospy.Subscriber("/duckiebot/joy",Joy,self.callback)
		self.publisher=rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd",Twist2DStamped,queue_size=1)
		self.twist=Twist2DStamped()


                
        def callback(self,msg):
		print msg.axes
		print msg.buttons
		if msg.buttons[0]==1:
			x1=msg.axes[0]
			x2=msg.axes[3]
			print x2
			if x1>0.9:
				self.twist.v=5
			if x2>0.9:
				self.twist.omega=-5
			if x1<-0.9:
				self.twist.v=-5
			if x2<-0.9:
				self.twist.omega=-5
		else:
			self.twist.v=0
			self.twist.omega=0
		#if msg.buttons[1]==1:
		print self.twist.v
			
		self.publisher.publish(self.twist)
		



def main():
	rospy.init_node('test') #creacion y registro del nodo!
	c=controller()
	#obj = controller('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objetc.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
