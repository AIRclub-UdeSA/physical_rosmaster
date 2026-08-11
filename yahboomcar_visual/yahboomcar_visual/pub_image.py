#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image



class PubImage(Node):
	def __init__(self,name):
		super().__init__(name)
		self.bridge = CvBridge()
		self.sub_img = self.create_subscription(Image,'/image_raw',self.handleTopic,500)
		self.pub_img = self.create_publisher(Image,'/image',500)


	def handleTopic(self,msg):
		if not isinstance(msg, Image):
			return
		frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		# 规范输入图像大小
		# Standardize the input image size
		frame = cv.resize(frame, (640, 480))
		# opencv mat ->  ros msg
		msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
		self.pub_img.publish(msg)


def main():
	rclpy.init()
	pub_image = PubImage('pub_image_node')
	rclpy.spin(pub_image)

