#!/usr/bin/env python3
# coding: utf-8
from Speech_Lib import Speech
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.clock import Clock
import rclpy
from rclpy.node import Node


class MarkNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 1)
        self.pub_cmdVel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.spe = Speech()
        # 创建定时器
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.voice_pub_goal)
        self.pose = PoseStamped()
    
    def voice_pub_goal(self):
        self.pose.header.frame_id = 'map'
        speech_r = self.spe.speech_read()
        # print("-------speech_r = ",speech_r)
        if speech_r == 19:
            print("goal to one")
            self.spe.void_write(speech_r)
            self.pose.header.stamp = Clock().now().to_msg()
            self.pose.pose.position.x = -7.1171722412109375
            self.pose.pose.position.y = -3.8613715171813965
            self.pose.pose.orientation.z = -0.6484729569092691
            self.pose.pose.orientation.w = 0.7612376922862854
            self.pub_goal.publish(self.pose)

        elif speech_r == 20:
            print("goal to two")
            self.spe.void_write(speech_r)
            self.pose.header.stamp = Clock().now().to_msg()
            self.pose.pose.position.x = -5.434411525726318
            self.pose.pose.position.y = -3.575838088989258
            self.pose.pose.orientation.z = 0.041131907433507836
            self.pose.pose.orientation.w = 0.9991537250047569
            self.pub_goal.publish(self.pose)

        elif speech_r == 21:
            print("goal to three")
            self.spe.void_write(speech_r)
            self.pose.header.stamp = Clock().now().to_msg()
            self.pose.pose.position.x = -5.734817981719971
            self.pose.pose.position.y = -2.142876148223877
            self.pose.pose.orientation.z = 0.7319496758927427
            self.pose.pose.orientation.w = 0.681358695519848
            self.pub_goal.publish(self.pose)

        elif speech_r == 32:
            print("goal to four")
            self.spe.void_write(speech_r)
            self.pose.header.stamp = Clock().now().to_msg()
            self.pose.pose.position.x = -7.1068596839904785
            self.pose.pose.position.y = -1.9793033599853516
            self.pose.pose.orientation.z = -0.9999801865013679
            self.pose.pose.orientation.w = 0.006294966615423905
            self.pub_goal.publish(self.pose)

        elif speech_r == 33:
            print("goal to Origin")
            self.spe.void_write(speech_r)
            self.pose.header.stamp = Clock().now().to_msg()
            self.pose.pose.position.x = -6.264475345611572
            self.pose.pose.position.y = -2.9691827297210693
            self.pose.pose.orientation.z = -0.6871058740679257
            self.pose.pose.orientation.w = 0.7265573052563382
            self.pub_goal.publish(self.pose)
        elif speech_r == 0:
            self.pub_cmdVel.publish(Twist())


def main():
    rclpy.init()
    markNode = MarkNode('voice_pub_goal_publisher')
    rclpy.spin(markNode)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()