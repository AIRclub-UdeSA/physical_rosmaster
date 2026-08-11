#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#commom lib
import math
import numpy as np
import time
from time import sleep
from yahboomcar_laser.common import *
print ("improt done")
RAD2DEG = 180 / math.pi

class laserAvoid(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan,1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)
        #create a pub
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
        
        #declareparam
        self.declare_parameter("linear",0.5)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular",1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle",40.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.55)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("Switch",False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.ros_ctrl = SinglePID()
        
        self.timer = self.create_timer(0.01,self.on_timer)
        
    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        ranges = np.array(scan_data.ranges)
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if -10 > angle > -10-self.LaserAngle:
                if ranges[i] < self.ResponseDist*1.5:
                    self.Right_warning += 1
            if 10+self.LaserAngle > angle > 10:
                if ranges[i] < self.ResponseDist*1.5:
                    self.Left_warning += 1
            if abs(angle) < 10:
                 if ranges[i] <= self.ResponseDist*1.5: 
                        self.front_warning += 1
        if self.Joy_active or self.Switch == True:
            if self.Moving == True:
                self.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return
        self.Moving = True
        twist = Twist()
        if self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning > 10:
            print ('1, there are obstacles in the left and right, turn right')
            twist.linear.x = self.linear
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        
        elif self.front_warning > 10 and self.Left_warning <= 10 and self.Right_warning > 10:
            print ('2, there is an obstacle in the middle right, turn left')
            twist.linear.x = self.linear
            twist.angular.z = self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
            if self.Left_warning > 10 and self.Right_warning <= 10:
                twist.linear.x = self.linear
                twist.angular.z = -self.angular
                self.pub_vel.publish(twist)
                sleep(0.5)
        elif self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            print ('4. There is an obstacle in the middle left, turn right')
            twist.linear.x = self.linear
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
            if self.Left_warning <= 10 and self.Right_warning > 10:
                twist.linear.x = self.linear
                twist.angular.z = self.angular
                self.pub_vel.publish(twist)
                sleep(0.5)
        elif self.front_warning > 10 and self.Left_warning < 10 and self.Right_warning < 10:

            print ('6, there is an obstacle in the middle, turn left')
            twist.linear.x = self.linear
            twist.angular.z = self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning > 10:
            print ('7. There are obstacles on the left and right, turn right')
            twist.linear.x = self.linear
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.4)
        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            print ('8, there is an obstacle on the left, turn right')
            twist.linear.x = self.linear
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning < 10 and self.Left_warning <= 10 and self.Right_warning > 10:
            print ('9, there is an obstacle on the right, turn left')
            twist.linear.x = self.linear
            twist.angular.z = self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning <= 10 and self.Left_warning <= 10 and self.Right_warning <= 10:
            print ('10, no obstacles, go forward')
            twist.linear.x = self.linear
            twist.angular.z = 0.0
            self.pub_vel.publish(twist)

def main():
    rclpy.init()
    laser_avoid = laserAvoid("laser_Avoidance_a1")
    print ("start it")
    rclpy.spin(laser_avoid)