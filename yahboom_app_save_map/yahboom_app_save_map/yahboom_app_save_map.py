from yahboom_web_savmap_interfaces.srv import WebSaveMap                                                            

import rclpy
from rclpy.node import Node
import sqlite3
import uuid
import subprocess
import datetime
import hashlib

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(WebSaveMap, 'yahboomAppSaveMap', self.add_three_ints_callback)       # CHANGE
        
    def run_shellcommand(self, *args):
        '''run the provided command and return its stdout'''
        args = sum([(arg if type(arg) == list else [arg]) for arg in args], [])
        print(args)
        possess = subprocess.Popen(args, stdout=subprocess.PIPE)
        return possess

    def add_three_ints_callback(self, request, response):
        map_name = request.mapname
        map_path = "/home/yahboom/cartoros2/data/maps/" + map_name
        now = datetime.datetime.now()
        str_time = now.strftime("%Y-%m-%d %H:%M:%S.%f")
        map_namestr = str_time + map_name
        map_id = hashlib.md5(map_namestr.encode()).hexdigest()
        response.response = request.mapname
        try:
            conn = sqlite3.connect("/home/yahboom/cartoros2/data/xgo.db")
            c = conn.cursor()
            c.execute("INSERT INTO xgo_map (map_name, map_id, map_path) VALUES (?, ?, ?)",(map_name, map_id, map_path))
            self.run_shellcommand('ros2', 'run', 'nav2_map_server', 'map_saver_cli','-f', map_path, '--ros-args', '-p', 'save_map_timeout:=10000')
        except sqlite3.IntegrityError as e:
            re_data = {"msg": str(e)}
            response.response = re_data
            self.get_logger().info('Incoming request\nmapname: %s' % (re_data))  # CHANGE
            return response     
                                                        # CHANGE
        self.get_logger().info('Incoming request\nmapname: %s' % (request.mapname))  # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
