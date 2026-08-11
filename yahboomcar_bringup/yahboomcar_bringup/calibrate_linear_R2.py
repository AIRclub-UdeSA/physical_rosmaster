import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import copysign, sqrt, pow
#import time
from rclpy.duration import Duration
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import PyKDL
from math import pi
from yahboomcar_bringup.transform_utils import *

class CalibrateLinear(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a spublisher
        self.cmd_vel = self.create_publisher(Twist,"/cmd_vel",5)
        #declare_parameter
        self.declare_parameter('rate',20.0)
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        
        self.declare_parameter('test_distance',1.0)
        self.test_distance = self.get_parameter('test_distance').get_parameter_value().double_value
        
        self.declare_parameter('speed',0.5)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        
        self.declare_parameter('tolerance',0.03)
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        
        self.declare_parameter('odom_linear_scale_correction',1.0)
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value
        
        self.declare_parameter('start_test',False)
        
        
        self.declare_parameter('direction',True)
        self.direction = self.get_parameter('direction').get_parameter_value().bool_value
        
        self.declare_parameter('base_frame','base_footprint')
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        self.declare_parameter('odom_frame','odom')
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        
        
        #init the tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rclpy.time.Time(), timeout = Duration(seconds = 60.0))
        #time.sleep(2)
        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y
        
        print ("finish init work")
        now = rclpy.time.Time()
        #trans = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now,timeout=Duration(seconds = 10.0))               
        #create timer 
        self.timer = self.create_timer(0.05, self.on_timer)


        
    def on_timer(self):
        move_cmd = Twist()
        #self.get_param()
        self.start_test = self.get_parameter('start_test').get_parameter_value().bool_value
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.test_distance = self.get_parameter('test_distance').get_parameter_value().double_value
        self.direction = self.get_parameter('direction').get_parameter_value().double_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        if self.start_test:
            '''trans = self.tf_buffer.lookup_transform(
                        self.odom_frame,
                        self.base_frame,
                        now,
                        )'''
            #self.position.x = trans.transform.translation.x
            #self.position.y = trans.transform.translation.y
            self.position.x = self.get_position().transform.translation.x
            self.position.y = self.get_position().transform.translation.y
            print("self.position.x: ",self.position.x)
            print("self.position.y: ",self.position.y)
            distance = sqrt(pow((self.position.x - self.x_start), 2) +
                                pow((self.position.y - self.y_start), 2))
            distance *= self.odom_linear_scale_correction
            print("distance: ",distance)
            error = distance - self.test_distance
            print("error: ",error)
            #start = time()
            if not self.start_test or abs(error) < self.tolerance:
                self.start_test  = rclpy.parameter.Parameter('start_test',rclpy.Parameter.Type.BOOL,False)
                all_new_parameters = [self.start_test]
                self.set_parameters(all_new_parameters)
                    
                print("done")
            else:
                move_cmd.linear.x = copysign(self.speed, -1 * error)
                '''if self.direction:
                    print("x")
                    move_cmd.linear.x = copysign(self.speed, -1 * error)
                else:
                    move_cmd.linear.y = copysign(self.speed, -1 * error)
                    print("y")'''
            self.cmd_vel.publish(move_cmd)
            #end = time()
        else:
            #self.position.x = trans.transform.translation.x
            #self.position.y = trans.transform.translation.y
            self.x_start = self.get_position().transform.translation.x
            self.y_start = self.get_position().transform.translation.y
            print("self.x_start: ",self.x_start)
            print("self.y_start:",self.y_start)
            self.cmd_vel.publish(Twist())
        #self.cmd_vel.publish(Twist() )    
        
    def get_param(self):
        #self.start_test = self.get_parameter('start_test').get_parameter_value().bool_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.test_distance = self.get_parameter('test_distance').get_parameter_value().double_value
        self.direction = self.get_parameter('direction').get_parameter_value().double_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
          
     
    def get_position(self):
         try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now)   
            return trans       
         except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            raise
            return
         
def main():
    rclpy.init()
    class_calibratelinear = CalibrateLinear("calibrate_linear")
    rclpy.spin(class_calibratelinear)
