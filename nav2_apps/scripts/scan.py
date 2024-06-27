

import os

import time
import math
import threading
from math import cos, sin, pi
from copy import deepcopy
# ----- #
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# ----- #
from sensor_msgs.msg import LaserScan

# ----- #
from std_srvs.srv import Empty

'''
Este codigo es la segunda parte del proceso automatico, una vez se haya encontrado
el cart_laser_frame debido al servidor server_patrol_behavior. Se procede a acercarse
a un punto de referencia para luego entrar al shelf, levantarlo y cambiar el footprint
'''

from ament_index_python.packages import get_package_share_directory

class AppService(Node):
    def __init__(self):
        super().__init__('App_Node')

        self.get_logger().info('Cart Frame Publisher Server is READY!')
        
        self.subscription = self.create_subscription(
                                    LaserScan,
                                    'scan',
                                    self.scan_callback,
                                    10)


        self.ranges = []
        self.i= 0

    def scan_callback(self, msg):

        if self.i == 0:
            self.ranges = msg.ranges
            print(f"Size = [{len(self.ranges)}]")
            print(self.ranges)
            self.i += 1

def main(args=None):
    rclpy.init(args=args)

    app_server_node_ = AppService()

    executor = MultiThreadedExecutor()
    executor.add_node(app_server_node_)


    try:
        executor.spin()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
