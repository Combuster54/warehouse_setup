
#! /usr/bin/env python3


"""
Ready
"""

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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# ----- #
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Polygon, Point32, Twist, PoseStamped
# from attach_shelf.srv import GoToLoading
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# ----- #
from std_srvs.srv import Empty

"""
Funciona de maravilla

"""
'''
Este codigo es la segunda parte del proceso automatico, una vez se haya encontrado
el cart_laser_frame debido al servidor server_patrol_behavior. Se procede a acercarse
a un punto de referencia para luego entrar al shelf, levantarlo y cambiar el footprint
'''

from ament_index_python.packages import get_package_share_directory

###### POSITIONS ######
####################
request_init_position = 'init_position'
request_first_waypoint = 'first_waypoint'
request_second_waypoint = 'second_waypoint'
request_third_waypoint = 'third_waypoint'
request_loading_position = 'loading_position'
request_shipping_position = 'shipping_position'
####################

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
import tf_transformations

from nav2_apps.move_shelf_class import MoveShelfNode
from nav2_apps.footprint_publisher_class import FootPrintPublisher
from nav2_apps.navigation2_class import Navigation
from nav2_apps.follow_cart_frame_class import FollowCartFrame
from nav2_apps.static_frame_publisher_class import StaticFramePublisher
from nav2_apps.frame_listener_class import FrameListener


class FramePublisher(Node):
    def __init__(self):
        super().__init__('FramePublisher_Node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.broadcaster = StaticTransformBroadcaster(self)

        self.timer = None
        self.detect = False

        # Transformation settings
        self.static_transform = TransformStamped()
        self.static_transform.header.frame_id = 'robot_cart_laser'
        self.static_transform.child_frame_id = 'approach_point'

         # Transformation settings
        self.load_shelf_frame = TransformStamped()
        self.load_shelf_frame.header.frame_id = 'robot_cart_laser'
        self.load_shelf_frame.child_frame_id = 'load_shelf_point'

        self.timer_on()
        self.get_logger().info("Frame Publisher ready")


    def timer_on(self):
        self.timer = self.create_timer(1.1, self.publish_frame)
        self.get_logger().info("oN")

    def timer_off(self):
        if self.timer:
            self.timer.cancel()

    def publish_frame(self):
        self.get_logger().info("Publish Frame")

        try:
            now = self.get_clock().now()
            timeout = Duration(seconds=1)
            transform = self.tf_buffer.lookup_transform('map', 'robot_cart_laser', Time())

            # Decide which side the shelf is on and set the appropriate transform
            if transform.transform.translation.y > 0.2:
                # Shelf on the left side
                self.set_transform(0.7, 0.0, 0.0)
                self.get_logger().info("canTransform: left")
                self.detect = True
            elif transform.transform.translation.y <= -0.3:
                # Shelf on the right side
                self.set_transform(-0.7, 0.0, 0.0)
                self.get_logger().info("canTransform: right")
                self.detect = True
            else:
                self.get_logger().info("canTransform: FALSE")

            #publish load_shelf_point 
            self.set_load_shelf()

            self.broadcaster.sendTransform(self.load_shelf_frame)
            self.broadcaster.sendTransform(self.static_transform)
            #self.timer_off()
        except TransformException as e:
            self.get_logger().warn(f'Transform error: {str(e)}')
            self.timer_off()

    def set_load_shelf(self):

        #publish load_shelf_point 
        self.load_shelf_frame.header.stamp = self.get_clock().now().to_msg()
        self.load_shelf_frame.transform.translation.x =  0.5
        self.load_shelf_frame.transform.translation.y = 0.0
        self.load_shelf_frame.transform.translation.z = 0.0
        self.load_shelf_frame.transform.rotation.x =    0.0
        self.load_shelf_frame.transform.rotation.y =  0.0
        self.load_shelf_frame.transform.rotation.z =   0.0
        self.load_shelf_frame.transform.rotation.w =  1.0

    def set_transform(self, x, y, z):
        self.static_transform.header.stamp = self.get_clock().now().to_msg()
        self.static_transform.transform.translation.x = x
        self.static_transform.transform.translation.y = y
        self.static_transform.transform.translation.z = z
        self.static_transform.transform.rotation.x =  0.0
        self.static_transform.transform.rotation.y =  0.0
        self.static_transform.transform.rotation.z =  0.0
        self.static_transform.transform.rotation.w =  1.0


def main(args=None):
    rclpy.init(args=args)

    client_node = FramePublisher()


    executor = MultiThreadedExecutor()

    executor.add_node(client_node)


    try:
        executor.spin()#Where I put the spin? o I just use while instead?
    except KeyboardInterrupt :
        #Is it possible to have a general get_logger? how?
        print("[ERROR]")

    #There are a lot of nodes!
    client_node.destroy_node()

    return None


if __name__ == '__main__':
    main()
