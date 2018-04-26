#!/usr/bin/env python

import rospy #importar ros para python
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Point
fx=162.16
fy=164
cx=162.94
cy=120
dpx=3.5
dpy=3.0
dpeligro=10
class duckfinder(object):
	def __init__(self):
		super(duckfinder, self).__init__() 
		self.subscriber=rospy.Subscriber("/duckiebot/camera_node/image/rect",Image,self.callback)
		#self.publisher=rospy.Publisher("/usb_cam/mask", Image,queue_size=1) 
		#self.publisher2=rospy.Publisher("/usb_cam/duck", Image,queue_size=1)
		#self.publisher3=rospy.Publisher("/usb_cam/t1", Image,queue_size=1)
		self.publisher4=rospy.Publisher("/usb_cam/rect", Image,queue_size=1)
		self.publisher=rospy.Publisher("posicion", Point,queue_size=1)
		self.bridge=CvBridge()
		self.point=Point()
		


                
        def callback(self,msg):
		image=self.bridge.imgmsg_to_cv2(msg,"bgr8")
		color_space= cv2.COLOR_BGR2HSV
		image_out=cv2.cvtColor(image, color_space)
		
		mask= cv2.inRange(image_out,np.array([15,150,150]),np.array([45,255,255]))		
		msg=self.bridge.cv2_to_imgmsg(mask,"mono8")
		image_out=cv2.bitwise_and(image,image,mask=mask)
		msg2=self.bridge.cv2_to_imgmsg(image_out,"bgr8")
		kernel= np.ones((5,5),np.uint8)
		image_out2=cv2.erode(image_out,kernel,iterations=1)
		msg3=self.bridge.cv2_to_imgmsg(image_out2,"bgr8")
		(_,contours, hierarchy) = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		image_out3=image.copy()
		L=[0,0,0]
		for pic, contour in enumerate(contours):
			area=cv2.contourArea(contour)
			if (area>100):
				x,y,w,h = cv2.boundingRect(contour)
				image_out3=cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 2)
				u=x+w/2.0
				v=y+h/2.0
				z=(fy*dpy*1.0)/h
				x3d=((u-cx)*z)/fx
				y3d=((v-cy)*z)/fy
				L=[x3d,y3d,z]
				self.point.x=x3d
				self.point.y=y3d
				self.point.z=z
		self.publisher.publish(self.point)
		msg4=self.bridge.cv2_to_imgmsg(image_out3,"bgr8")
		self.publisher4.publish(msg4)
		#self.publisher.publish(msg)
		#self.publisher2.publish(msg2)
		#self.publisher3.publish(msg3)
		#self.publisher4.publish(msg4)
		



def main():
	rospy.init_node('test') #creacion y registro del nodo!
	c=duckfinder()
	#obj = controller('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objetc.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
