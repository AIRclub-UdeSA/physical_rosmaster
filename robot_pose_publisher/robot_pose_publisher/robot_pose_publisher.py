#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math

class RobotPosePublisher(Node):

    def __init__(self):
        super().__init__('robot_pose_publisher')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.odom_callback,
            10)
       
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.robot_pose_publisher = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            10)
        timer_period = 2  #2s的定时间隔
        self.timer = self.create_timer(timer_period, self.robot_pose_callback)
        self.robot_pose = PoseStamped()
        
    def odom_callback(self, msg):
        try:
            self.robot_pose.header.frame_id = 'map'
            self.robot_pose.header.stamp = self.get_clock().now().to_msg()
            self.robot_pose.pose.position.x = msg.pose.pose.position.x
            self.robot_pose.pose.position.y = msg.pose.pose.position.y
            self.robot_pose.pose.position.z = msg.pose.pose.position.z
            self.robot_pose.pose.orientation.x = msg.pose.pose.orientation.x
            self.robot_pose.pose.orientation.y = msg.pose.pose.orientation.y
            self.robot_pose.pose.orientation.z = msg.pose.pose.orientation.z
            self.robot_pose.pose.orientation.w = msg.pose.pose.orientation.w
            self.robot_pose_publisher.publish(self.robot_pose)
        except :
            self.get_logger().warn("msg err")

    def robot_pose_callback(self):
        self.robot_pose_publisher.publish(self.robot_pose)

    
def main(args=None):
    rclpy.init(args=args)
    robot_pose_publisher = RobotPosePublisher()
    rclpy.spin(robot_pose_publisher)
    robot_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

