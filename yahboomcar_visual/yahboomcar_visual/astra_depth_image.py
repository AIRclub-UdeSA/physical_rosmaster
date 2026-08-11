import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

encoding = ['16UC1', '32FC1']

class AstraDepthImage(Node):
	def __init__(self,name):
		super().__init__(name)
		self.bridge = CvBridge()
		self.sub_img = self.create_subscription(Image,'/camera/depth/image_raw',self.handleTopic,10)

	def handleTopic(self,msg):
		if not isinstance(msg, Image):
			
			return
		frame = self.bridge.imgmsg_to_cv2(msg, "32FC1")
		# 规范输入图像大小
		# Standardize the input image size
		frame = cv.resize(frame, (640, 480))
		# opencv mat ->  ros msg
		cv.imshow("depth_image", frame)
		
		cv.waitKey(10)

def main():
	rclpy.init()
	get_astra_depth = AstraDepthImage('get_astra_depth_node')
	rclpy.spin(get_astra_depth)


