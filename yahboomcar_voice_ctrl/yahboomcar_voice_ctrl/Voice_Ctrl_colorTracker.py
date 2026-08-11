#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from yahboomcar_msgs.msg import Position
#common lib
import os
import threading
import math
from yahboomcar_astra.astra_common import *
from yahboomcar_msgs.msg import Position
from cv_bridge import CvBridge
print("import done")

class color_Tracker(Node):
    def __init__(self,name):
        super().__init__(name)
        #create the publisher
        self.pub_cmdVel = self.create_publisher(Twist,'/cmd_vel',10)
        #create the subscriber
        self.sub_depth = self.create_subscription(Image,"/camera/depth/image_raw", self.depth_img_Callback, 1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState',  self.JoyStateCallback,1)
        self.sub_position = self.create_subscription(Position,"/Current_point",self.positionCallback,1)
        self.bridge = CvBridge()
        self.minDist = 1500
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.Center_prevx = 0
        self.Center_prevr = 0
        self.prev_time = 0
        self.prev_dist = 0
        self.prev_angular = 0
        self.Joy_active = True
        self.Robot_Run = False
        self.dist = []
        self.encoding = ['16UC1', '32FC1']
        self.linear_PID = (3.0, 0.0, 1.0)
        self.angular_PID = (0.5, 0.0, 2.0)
        self.scale = 1000
        self.PID_init()
        self.declare_param()
        print("init done")
        
    def declare_param(self):
        self.declare_parameter("linear_Kp",3.0)
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.declare_parameter("linear_Ki",0.0)
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.declare_parameter("linear_Kd",1.0)
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.declare_parameter("angular_Kp",0.5)
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.declare_parameter("angular_Ki",0.0)
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.declare_parameter("angular_Kd",2.0)
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.declare_parameter("scale",1000)
        self.scale = self.get_parameter('scale').get_parameter_value().integer_value
        self.declare_parameter("minDistance",1.0)
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        
    def get_param(self):
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.scale = self.get_parameter('scale').get_parameter_value().integer_value
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        
        self.linear_PID = (self.linear_Kp, self.linear_Ki, self.linear_Kd)
        self.angular_PID = (self.angular_Kp, self.angular_Ki, self.angular_Kd)
        self.minDist = self.minDistance * 1000
        

    def PID_init(self):
        self.linear_pid = simplePID(self.linear_PID[0] / 1000.0, self.linear_PID[1] / 1000.0, self.linear_PID[2] / 1000.0)
        self.angular_pid = simplePID(self.angular_PID[0] / 100.0, self.angular_PID[1] / 100.0, self.angular_PID[2] / 100.0)
        
    def depth_img_Callback(self, msg):
        if not isinstance(msg, Image): return
        depthFrame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding[1])
        self.action = cv.waitKey(1)
        if self.Center_r != 0:
            now_time = time.time()
            if now_time - self.prev_time > 5:
                if self.Center_prevx == self.Center_x and self.Center_prevr == self.Center_r: self.Center_r = 0
                self.prev_time = now_time
            distance = [0, 0, 0, 0, 0]
            if 0 < int(self.Center_y - 3) and int(self.Center_y + 3) < 480 and 0 < int(
                self.Center_x - 3) and int(self.Center_x + 3) < 640:
                # print("depthFrame: ", len(depthFrame), len(depthFrame[0]))
                distance[0] = depthFrame[int(self.Center_y - 3)][int(self.Center_x - 3)]
                distance[1] = depthFrame[int(self.Center_y + 3)][int(self.Center_x - 3)]
                distance[2] = depthFrame[int(self.Center_y - 3)][int(self.Center_x + 3)]
                distance[3] = depthFrame[int(self.Center_y + 3)][int(self.Center_x + 3)]
                distance[4] = depthFrame[int(self.Center_y)][int(self.Center_x)]
                distance_ = 1000.0
                num_depth_points = 5
                for i in range(5):
                    if 40 < distance[i] < 80000: distance_ += distance[i]
                    else: num_depth_points -= 1
                if num_depth_points == 0: distance_ = self.minDist
                else: distance_ /= num_depth_points
                #print("Center_x: {}, Center_y: {}, distance_: {}".format(self.Center_x, self.Center_y, distance_))
                self.execute(self.Center_x, distance_)
                self.Center_prevx = self.Center_x
                self.Center_prevr = self.Center_r
        else:
            if self.Robot_Run ==True:
                self.pub_cmdVel.publish(Twist())
                self.Robot_Run = False
        if self.action == ord('q') or self.action == 113: self.cleanup()
        
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        self.pub_cmdVel.publish(Twist())
        
    def positionCallback(self, msg):
        if not isinstance(msg, Position): return
        self.Center_x = msg.anglex
        self.Center_y = msg.angley
        self.Center_r = msg.distance
        
    def cleanup(self):
        self.pub_cmdVel.publish(Twist())
        print ("Shutting down this node.")
        cv.destroyAllWindows()
        
    def execute(self, point_x, dist):
        self.get_param()
        if abs(self.prev_dist - dist) > 300:
            self.prev_dist = dist
            return
        if abs(self.prev_angular - point_x) > 300:
            self.prev_angular = point_x
            return
        if self.Joy_active == True: return
        linear_x = self.linear_pid.compute(dist, self.minDist)
        angular_z = self.angular_pid.compute(320, point_x)
        if abs(dist - self.minDist) < 30: linear_x = 0
        if abs(point_x - 320.0) < 30: angular_z = 0
        twist = Twist()
        if angular_z>2.0:
            angular_z = 2.0
        if angular_z<-2.0:
            angular_z = -2.0
        if linear_x > 1.0:
            linear_x = 1.0
        if linear_x <-1.0:
            linear_x = -1.0
        twist.angular.z = angular_z * 1.0
        twist.linear.x = linear_x * 1.0
        print("twist.linear.x: ",twist.linear.x)
        print("twist.angular.z: ",twist.angular.z)
        self.pub_cmdVel.publish(twist)
        self.Robot_Run = True
        
def main():
    rclpy.init()
    color_tracker = color_Tracker("ColorTracker")
    print("start it")
    rclpy.spin(color_tracker)
