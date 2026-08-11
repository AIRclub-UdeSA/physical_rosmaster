#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

class Image_Flip(Node):
	def __init__(self,name):
		super().__init__(name)
		self.sub_img = self.create_subscription(CompressedImage,'/image_raw/compressed',self.handleTopic,1)
		self.pub_comimg = self.create_publisher(CompressedImage,'/image_flip/compressed',1)
		self.bridge = CvBridge()
		
	def handleTopic(self,msg):
		if not isinstance(msg, CompressedImage):
			return
					
		frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
		frame = cv.resize(frame, (640, 480))
		frame = cv.flip(frame, 1)
		cv.imshow("flip_image", frame)
		cv.waitKey(10)
		msg = CompressedImage()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.data = np.array(cv.imencode('.jpg', frame)[1]).tostring()
		self.pub_comimg.publish(msg)

def main():
	rclpy.init()
	image_flip = Image_Flip('get_astra_rgb_node')
	rclpy.spin(image_flip)

