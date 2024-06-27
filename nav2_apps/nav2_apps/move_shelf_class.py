#! /usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MoveShelfNode(Node):
    """
    MoveShelfNode.

    MoveShelfNode Class: This class handles the function of lifting and descending the 
                         elevator, using the publishers

    Functions:

        @publish_message_down: publisher to down robot's elevator
        @publish_message_up:   publisher to lift up robot's elevator

    """
    def __init__(self, node_name='move_shelf_node', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)

        self.publisher_down_ = self.create_publisher(String, '/elevator_down', 10)
        self.publisher_up_ = self.create_publisher(String, '/elevator_up', 10)
        self.elevator_msg = String()
        self.elevator_msg.data = ""   # this will always be blank string!
        return None

    def publish_message_down(self):
        i = 0 
        while(i<3):
            self.publisher_down_.publish(self.elevator_msg)
            self.get_logger().info(f'[Down-Elevator] Publishing...')
            time.sleep(1.0)
            i+=1
        return None

    def publish_message_up(self):
        i = 0 
        while(i<3):
            self.publisher_up_.publish(self.elevator_msg)
            self.get_logger().info(f'[Up-Elevator] Publishing...')
            time.sleep(1.0)
            i+=1

        return None