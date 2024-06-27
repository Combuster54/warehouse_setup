#! /usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from rclpy.executors import MultiThreadedExecutor

class MoveShelfNode(Node):
    """
    MoveShelfNode.

    MoveShelfNode Class: This class handles the function of lifting and descending the 
                         elevator, using the publishers

    Functions:

        @publish_message_down: publisher to down robot's elevator
        @publish_message_up:   publisher to lift up robot's elevator

    """
    def __init__(self):
        super().__init__('MoveShelf_Node')
        self.publisher_down_ = self.create_publisher(String, '/elevator_down', 10)
        self.publisher_up_ = self.create_publisher(String, '/elevator_up', 10)
        self.elevator_msg = String()
        self.elevator_msg.data = ""   # this will always be blank string!
        # self.publish_message_up()
        self.publish_message_down()
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


def main():

    rclpy.init()

    shelf_position_node_ = MoveShelfNode()

    executor = MultiThreadedExecutor()

    executor.add_node(shelf_position_node_)


    try:
        print("### MoveShelf Initialize ###")
        executor.spin()#Where I put the spin? o I just use while instead?
    except KeyboardInterrupt :
        #Is it possible to have a general get_logger? how?
        print("[ERROR]")
        shelf_position_node_.get_logger().info('Keyboard interrupt, shutting down.\n')


    shelf_position_node_.destroy_node()

    rclpy.shutdown()

    return None

if __name__ == '__main__':
    main()