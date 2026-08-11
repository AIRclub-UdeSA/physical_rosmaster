#!/usr/bin/env python
# coding:utf-8
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.clock import Clock


class scan_compression(Node):
    def __init__(self,name):
        super().__init__(name)
        self.multiple = 2
        self.pub = self.create_publisher(LaserScan, "/downsampled_scan", 1000)
        self.laserSub = self.create_subscription(LaserScan,"/scan", self.laserCallback, 1000)

    def laserCallback(self, data):
        # self.get_logger().info("laserCallback")
        if not isinstance(data, LaserScan): return
        laser_scan = LaserScan()
        laser_scan.header.stamp = Clock().now().to_msg()
        laser_scan.header.frame_id = data.header.frame_id
        laser_scan.angle_increment = data.angle_increment * self.multiple
        laser_scan.time_increment = data.time_increment
        laser_scan.intensities = data.intensities
        laser_scan.scan_time = data.scan_time
        laser_scan.angle_min = data.angle_min
        laser_scan.angle_max = data.angle_max
        laser_scan.range_min = data.range_min
        laser_scan.range_max = data.range_max
        # self.get_logger().info("len(np.array(data.ranges)) = {}".format(len(np.array(data.ranges))))
        for i in range(len(np.array(data.ranges))):
            if i % self.multiple == 0: laser_scan.ranges.append(data.ranges[i])
        self.pub.publish(laser_scan)

def main():
    rclpy.init()
    scan_cp = scan_compression("scan_dilute")
    rclpy.spin(scan_cp)
    scan_cp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
