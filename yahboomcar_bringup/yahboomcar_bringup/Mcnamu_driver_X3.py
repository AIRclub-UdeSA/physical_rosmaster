#!/usr/bin/env python
# encoding: utf-8

#public lib
import sys
import math
import random
import threading
from math import pi
from time import sleep
from Rosmaster_Lib import Rosmaster

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,MagneticField, JointState
from rclpy.clock import Clock

#from dynamic_reconfigure.server import Server
car_type_dic={
    'R2':5,
    'X3':1,
    'NONE':-1
}
class yahboomcar_driver(Node):
	def __init__(self, name):
		super().__init__(name)
		global car_type_dic
		self.RA2DE = 180 / pi
		self.car = Rosmaster()
		self.car.set_car_type(1)
		#get parameter
		self.declare_parameter('car_type', 'X3')
		self.car_type = self.get_parameter('car_type').get_parameter_value().string_value
		print (self.car_type)
		self.declare_parameter('imu_link', 'imu_link')
		self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
		print (self.imu_link)
		self.declare_parameter('Prefix', "")
		self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
		print (self.Prefix)
		self.declare_parameter('xlinear_limit', 1.0)
		self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
		print (self.xlinear_limit)
		self.declare_parameter('ylinear_limit', 1.0)
		self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
		print (self.ylinear_limit)
		self.declare_parameter('encoder_cpr', 1040.0)
		self.encoder_cpr = self.get_parameter('encoder_cpr').get_parameter_value().double_value

		# create subcriber
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
		self.sub_RGBLight = self.create_subscription(Int32,"RGBLight",self.RGBLightcallback,100)
		self.sub_BUzzer = self.create_subscription(Bool,"Buzzer",self.Buzzercallback,100)

		# create publisher
		self.EdiPublisher = self.create_publisher(Float32,"edition",100)
		self.volPublisher = self.create_publisher(Float32,"voltage",100)
		self.staPublisher = self.create_publisher(JointState,"joint_states",100)
		self.velPublisher = self.create_publisher(Twist,"vel_raw",50)
		self.imuPublisher = self.create_publisher(Imu,"/imu/data_raw",100)
		self.magPublisher = self.create_publisher(MagneticField,"/imu/mag",100)

		# create timer
		self.timer = self.create_timer(0.1, self.pub_data)

		# create and init variable
		self.edition = Float32()
		self.edition.data = 1.0
		self.prev_encoders = None
		self.prev_time = None
		self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # [fl, fr, bl, br]
		self.car.create_receive_threading()
	#callback function
	def cmd_vel_callback(self,msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
		if not isinstance(msg, Twist): return
        # 下发线速度和角速度
        # Issue linear vel and angular vel
		vx = msg.linear.x*1.0
        #vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
		vy = msg.linear.y*1.0
		angular = msg.angular.z*1.0     # wait for chang
		self.car.set_car_motion(vx, vy, angular)
		'''print("cmd_vx: ",vx)
		print("cmd_vy: ",vy)
		print("cmd_angular: ",angular)'''
        #rospy.loginfo("nav_use_rot:{}".format(self.nav_use_rotvel))
        #print(self.nav_use_rotvel)
	def RGBLightcallback(self,msg):
        # 流水灯控制，服务端回调函数 RGBLight control
		if not isinstance(msg, Int32): return
		# print ("RGBLight: ", msg.data)
		for i in range(3): self.car.set_colorful_effect(msg.data, 6, parm=1)
	def Buzzercallback(self,msg):
		if not isinstance(msg, Bool): return
		if msg.data:
			for i in range(3): self.car.set_beep(1)
		else:
			for i in range(3): self.car.set_beep(0)

	#pub data
	def pub_data(self):
		now = Clock().now()
		time_stamp = now.to_msg()
		imu = Imu()
		twist = Twist()
		battery = Float32()
		edition = Float32()
		mag = MagneticField()
		state = JointState()
		state.header.stamp = time_stamp
		state.header.frame_id = "joint_states"
		
		# Official X3 Mecanum joint names
		joint_base_names = ["front_left_joint", "front_right_joint", "back_left_joint", "back_right_joint"]
		if len(self.Prefix) == 0:
			state.name = joint_base_names
		else:
			state.name = [self.Prefix + j for j in joint_base_names]
		
		# Poll encoders from Rosmaster_Lib (m1: FR, m2: FL, m3: RR, m4: RL)
		try:
			m1, m2, m3, m4 = self.car.get_motor_encoder()
			# Map to [FL, FR, BL, BR]
			curr_encoders = [m2, m1, m4, m3]
			
			if self.prev_encoders is not None and self.prev_time is not None:
				dt = (now - self.prev_time).nanoseconds / 1e9
				if dt > 0:
					velocities = []
					rad_per_tick = (2.0 * math.pi) / max(1.0, self.encoder_cpr)
					for i in range(4):
						delta_ticks = curr_encoders[i] - self.prev_encoders[i]
						delta_rad = delta_ticks * rad_per_tick
						self.joint_positions[i] += delta_rad
						velocities.append(delta_rad / dt)
					state.position = list(self.joint_positions)
					state.velocity = velocities
				else:
					state.position = list(self.joint_positions)
					state.velocity = [0.0, 0.0, 0.0, 0.0]
			else:
				state.position = list(self.joint_positions)
				state.velocity = [0.0, 0.0, 0.0, 0.0]
			
			self.prev_encoders = curr_encoders
			self.prev_time = now
		except Exception:
			state.position = list(self.joint_positions)
			state.velocity = [0.0, 0.0, 0.0, 0.0]
			
		self.staPublisher.publish(state)
		
		#print ("mag: ",self.car.get_magnetometer_data())		
		edition.data = self.car.get_version()*1.0
		battery.data = self.car.get_battery_voltage()*1.0
		ax, ay, az = self.car.get_accelerometer_data()
		gx, gy, gz = self.car.get_gyroscope_data()
		mx, my, mz = self.car.get_magnetometer_data()
		mx = mx * 1.0
		my = my * 1.0
		mz = mz * 1.0
		vx, vy, angular = self.car.get_motion_data()
		'''print("vx: ",vx)
		print("vy: ",vy)
		print("angular: ",angular)'''
		# 发布陀螺仪的数据
		# Publish gyroscope data
		imu.header.stamp = time_stamp
		imu.header.frame_id = self.imu_link
		imu.linear_acceleration.x = ax*1.0
		imu.linear_acceleration.y = ay*1.0
		imu.linear_acceleration.z = az*1.0
		imu.angular_velocity.x = gx*1.0
		imu.angular_velocity.y = gy*1.0
		imu.angular_velocity.z = gz*1.0

		mag.header.stamp = time_stamp
		mag.header.frame_id = self.imu_link
		mag.magnetic_field.x = mx*1.0
		mag.magnetic_field.y = my*1.0
		mag.magnetic_field.z = mz*1.0
		
		# 将小车当前的线速度和角速度发布出去
		# Publish the current linear vel and angular vel of the car
		twist.linear.x = vx *1.0
		twist.linear.y = vy *1.0
		twist.angular.z = angular*1.0    
		self.velPublisher.publish(twist)
		self.imuPublisher.publish(imu)
		self.magPublisher.publish(mag)
		self.volPublisher.publish(battery)
		self.EdiPublisher.publish(edition)
		
		
			
def main():
	rclpy.init() 
	driver = yahboomcar_driver('driver_node')
	rclpy.spin(driver)

'''if __name__ == '__main__':
	main()'''

		
		
