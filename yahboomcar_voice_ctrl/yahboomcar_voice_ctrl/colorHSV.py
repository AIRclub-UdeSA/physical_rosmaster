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
from Speech_Lib import Speech
print("import finish")
cv_edition = cv.__version__
print("cv_edition: ",cv_edition)
class Color_Identify(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a publisher
        self.pub_position = self.create_publisher(Position,"/Current_point", 10)
        self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.index = 2
        self.Roi_init = ()
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.point_pose = (0, 0, 0)
        self.dyn_update = True
        self.Start_state = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.Track_state = 'identify'
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.declare_param()
        self.spe = Speech()
        self.model = "Gennal"
        self.hsv_text = "/root/yahboomcar_ros2_ws/yahboomcar_lucio_ws/src/yahboomcar_astra/yahboomcar_astra/colorHSV.text"
        self.capture = cv.VideoCapture(0)
        if cv_edition[0]=='3': self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        else: self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.timer = self.create_timer(0.001, self.on_timer)
        
        
        
    def declare_param(self):
        #HSV
        self.declare_parameter("Hmin",0)
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.declare_parameter("Smin",85)
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.declare_parameter("Vmin",126)
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.declare_parameter("Hmax",9)
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.declare_parameter("Smax",253)
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.declare_parameter("Vmax",253)
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
        self.declare_parameter('refresh',False)
        self.refresh = self.get_parameter('refresh').get_parameter_value().bool_value

        
    def on_timer(self):
        self.get_param()
        ret, frame = self.capture.read()
        action = cv.waitKey(10) & 0xFF
        frame, binary =self.process(frame, action)
        start = time.time()
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
        else:cv.imshow('frame', frame)
        if action == ord('q') or action == 113:
            self.capture.release()
            cv.destroyAllWindows()
            
    def process(self, rgb_img, action):
        
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        command_result = self.spe.speech_read()
        self.spe.void_write(command_result)
        if command_result == 73 :
        	self.model = "color_follow_line"
        	print("tracker red")
        	self.hsv_range =  [(0, 175, 149), (180, 253, 255)]

        elif command_result == 74 :
        	self.model = "color_follow_line"
        	print("tracker green")
        	self.hsv_range =  [(44, 138, 91), (84, 255, 255)]
 
        elif command_result == 75 :
        	self.model = "color_follow_line"   
        	print("tracker bule")
        	self.hsv_range =  [(83, 217, 196), (141, 253, 255)] 

        elif command_result == 72 :
        	self.model = "color_follow_line"
        	print("tracker yellow")
        	self.hsv_range =  [(18, 55, 187), (81, 253, 255)]
            
        elif command_result == 76 :
        	self.model = "Stop"
        	self.pub_cmdVel.publish(Twist())
        
        if  self.model == "color_follow_line":
        	rgb_img, binary, self.circle = self.color.object_follow(rgb_img, self.hsv_range)
        	if self.circle[2] != 0: threading.Thread(
                target=self.execute, args=(self.circle[0], self.circle[1], self.circle[2])).start()
        	if self.point_pose[0] != 0 and self.point_pose[1] != 0: threading.Thread(
                target=self.execute, args=(self.point_pose[0], self.point_pose[1], self.point_pose[2])).start()
            #threading.Thread(target=self.execute, args=(self.circle[0], self.circle[2])).start()
        return rgb_img, binary        

    
    def execute(self, x, y, z):
        position = Position()
        position.anglex = x * 1.0
        position.angley = y * 1.0
        position.distance = z * 1.0
        self.pub_position.publish(position)

    def get_param(self):
        #hsv
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
        self.refresh = self.get_parameter('refresh').get_parameter_value().bool_value
        
    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        for i in range(3): self.pub_position.publish(Position())
        print("succes!!!")
        
    def cancel(self):
        print("Shutting down this node.")
        cv.destroyAllWindows()
        
    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
        
        

def main():
    rclpy.init()
    color_identify = Color_Identify("ColorIdentify")
    print("start it")
    rclpy.spin(color_identify)
