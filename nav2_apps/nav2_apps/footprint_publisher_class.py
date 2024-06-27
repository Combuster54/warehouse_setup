#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from geometry_msgs.msg import Polygon, Point32, Twist



class FootPrintPublisher(Node):

    """
    FootPrintPublisher.

    FootPrintPublisher Class: This class handles the function of change footprint 
                              for local_costmap and global_costmap which are generated
                              by Nav2 config

    Functions:
        @publish_footprint_shelf: change the topics for a footprint with shelf + robot size
        @publish_init_footprint : restart the topics to default footprint
    """

    def __init__(self, node_name='footprint_publisher_node', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)


        self.pub_local = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.pub_global = self.create_publisher(Polygon, '/global_costmap/footprint', 10)

        self.publisher_down_ = self.create_publisher(String, '/elevator_down', 10)
        self.publisher_up_ = self.create_publisher(String, '/elevator_up', 10)

        self.elevator_msg = String()
        self.elevator_msg.data = ""   # this will always be blank string!
        return None
        
    def publish_footprint_shelf(self):

        footprint = Polygon()
        point1 = Point32()
        point1.x = 0.5
        point1.y = 0.4

        point2 = Point32()
        point2.x = 0.5
        point2.y = -0.4

        point3 = Point32()
        point3.x = -0.5
        point3.y = -0.4

        point4 = Point32()
        point4.x = -0.5
        point4.y = 0.4

        footprint.points = [point1, point2, point3, point4]

        self.pub_local.publish(footprint)
        self.pub_global.publish(footprint)
        self.get_logger().info(f"Publish Shelf FootPrint") 

        return None

    def publish_init_footprint(self):


        footprint = Polygon()
        point1 = Point32()
        point1.x = 0.25
        point1.y = 0.25

        point2 = Point32()
        point2.x = 0.25
        point2.y = -0.25

        point3 = Point32()
        point3.x = -0.25
        point3.y = -0.25

        point4 = Point32()
        point4.x = -0.25
        point4.y = 0.25

        footprint.points = [point1, point2, point3, point4]

        self.pub_local.publish(footprint)
        self.pub_global.publish(footprint)
        self.get_logger().info(f"Publish Init FootPrint") 

        return None