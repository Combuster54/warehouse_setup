#! /usr/bin/env python3
import time
from math import cos, sin, pi
from copy import deepcopy
# ----- #
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# ----- #

from std_srvs.srv import Empty
from geometry_msgs.msg import Transform
# from attach_shelf.srv import GoToLoading
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# ----- #

from ament_index_python.packages import get_package_share_directory

from nav2_apps.navigation2_class import Navigation
from nav2_apps.move_shelf_class import MoveShelfNode
from nav2_apps.footprint_publisher_class import FootPrintPublisher
from nav2_apps.static_frame_publisher_class import StaticFramePublisher

"""
Funciona, el punto es que hacer al momento de retroceder, ya que muchas veces la posicion de recojo no sera exacta.
Por otro lado, falta a;adilo como servidor, ahora solo esta como funcion
"""

"""
This code performs the following operations with the RB-1 robot:
1. Moves the RB-1 to the 'approach_unload_position'.
2. Navigates to the 'unload_position'.
3. Unloads the shelf.
4. Resets the footprint to the original size of the RB-1 robot.
5. Reverses to move away from the shelf and complete the operation.
"""

###### POSITIONS ######
####################
request_init_position = 'init_position'
request_approach_unload_position = 'approach_unload_position'
request_unload_position = 'unload_position'
####################

class ShelfPositionServer(Node):

    def __init__(self):
        super().__init__('shelf_position_server_node')

        #Nodes
        self.move_shelf_node_ = MoveShelfNode(namespace='shelf_position_server')
        self.footprint_publisher_node_ = FootPrintPublisher(namespace='shelf_position_server')
        self.navigation_node_ = Navigation()

        ## Setting Log level
        self.declare_parameter('position_log_level', 'info')
        log_level = self.get_parameter('position_log_level').get_parameter_value().string_value
        
        self.log_levels = {
            'debug': rclpy.logging.LoggingSeverity.DEBUG,
            'info': rclpy.logging.LoggingSeverity.INFO,
            'warn':rclpy.logging.LoggingSeverity.WARN,
            'error': rclpy.logging.LoggingSeverity.ERROR,
            'fatal': rclpy.logging.LoggingSeverity.FATAL
        }

        log_level = self.log_levels.get(log_level.lower(), rclpy.logging.LoggingSeverity.INFO)

        self.get_logger().set_level(log_level)
        self.move_shelf_node_.get_logger().set_level(log_level)
        self.footprint_publisher_node_.get_logger().set_level(log_level)
        self.navigation_node_.get_logger().set_level(log_level)

        # Transform
        self.unload_transform = Transform()
        # Position
        self.unload_transform.translation.x = 4.486
        self.unload_transform.translation.y = -1.650
        self.unload_transform.translation.z = 0.025
        # Orientation
        self.unload_transform.rotation.x = -0.000
        self.unload_transform.rotation.y = -0.000
        self.unload_transform.rotation.z = -0.747
        self.unload_transform.rotation.w = 0.665

        self.unload_shelf_flag = False

        self.static_frame_publisher = StaticFramePublisher(node_name='static_frame_publisher_node',namespace='shelf_position_server',source_frame='map',target_frame='unload_frame',position=self.unload_transform)
        #self.static_frame_publisher.timer_on()

    def unload_shelf(self):

        self.unload_shelf_flag = False
        #Approach
        self.navigation_node_.go_pose(request_approach_unload_position,'common_bt.xml') #which means go to this pose
        #go to left pose
        self.navigation_node_.go_pose(request_unload_position,'common_bt.xml') #which means go to this pose
        
        self.move_shelf_node_.publish_message_down()
        time.sleep(5)
        # Publish RB-1 footprint
        self.footprint_publisher_node_.publish_init_footprint()
        #backward movement
        self.navigation_node_.publish_velocity_backward()
        self.unload_shelf_flag = True


class GenericServer(Node):

    """
    GenericServer.

    GenericServer Class: Once you have robot_cart_laser, this class publish a 
                         reach approach with nav2, then use cmd_vel topic for final approeach
                         Finally, lift the object

    Change robot_base_frame && frame_to_follow for your own purposes

    Functions:

    Important Variables: 

    """

    def __init__(self):
        super().__init__('server_pub_frame')

        #Flag to see if the process has succeed
        self.isDone = False

        # Initialize FramePublisher Class
        self.shelf_position_node_ = ShelfPositionServer()

        self.srv = self.create_service(Empty, 'shelf_position_server', self.service_callback)
        self.get_logger().info('Approach frame server is READY!')

    def service_callback(self, request, response):
        
        self.get_logger().debug('[Shelf Position Server] Called!')
        self.shelf_position_node_.unload_shelf()
        if self.shelf_position_node_.unload_shelf_flag:
            self.get_logger().debug('[Shelf Position Server] SUCCESS!')
            return response

def main():

    rclpy.init()

    generic_server_node_ = GenericServer()

    shelf_position_node_ = generic_server_node_.shelf_position_node_

    navigation_node = shelf_position_node_.navigation_node_
    move_shelf_node = shelf_position_node_.move_shelf_node_
    footprint_publisher_node = shelf_position_node_.footprint_publisher_node_
    static_frame_publisher_node_ = shelf_position_node_.static_frame_publisher

    generic_server_node_.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    navigation_node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    move_shelf_node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    footprint_publisher_node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    static_frame_publisher_node_.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    executor = MultiThreadedExecutor()

    executor.add_node(generic_server_node_)
    executor.add_node(move_shelf_node)
    executor.add_node(footprint_publisher_node)
    executor.add_node(navigation_node)
    executor.add_node(static_frame_publisher_node_)

    try:
        print("### MoveShelf Initialize ###")
        executor.spin()#Where I put the spin? o I just use while instead?
    except KeyboardInterrupt :
        #Is it possible to have a general get_logger? how?
        print("[ERROR]")
        shelf_position_node_.get_logger().info('Keyboard interrupt, shutting down.\n')


    generic_server_node_.destroy_node()
    move_shelf_node.destroy_node()
    footprint_publisher_node.destroy_node()
    navigation_node.destroy_node()
    static_frame_publisher_node_.destroy_node()
    rclpy.shutdown()

    return None

if __name__ == '__main__':
    main()