#! /usr/bin/env python3

# ----- #
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# ----- #

from std_msgs.msg import Empty

import os
import time 

"""
Ready
Testear

Poner un manejo de error en destroy node, porque ya se destruyo el nodo previamente
"""

from rclpy.node import Node
from nav2_apps.navigation2_class import Navigation
from nav2_apps.frame_listener_class import FrameListener
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter


class costMapClients(Node):

    def __init__(self, node_name='costmap_client_node', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)

        self.local_costmap_cli = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        self.global_costmap_cli = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')

        while not self.local_costmap_cli.wait_for_service(timeout_sec=1.0) and not  self.global_costmap_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('services not available yet, waiting again...')
        self.req = SetParameters.Request()
        self.get_logger().info(f'{namespace}/{node_name} ready')

    def send_request(self):
        self.get_logger().info('Sending request...')       

        #Shelf Server POsition
        self.req.parameters = [
            Parameter(name='inflation_layer.inflation_radius', value=0.42).to_parameter_msg(),
            Parameter(name='inflation_layer.cost_scaling_factor', value = 5.0).to_parameter_msg(),
            Parameter(name='obstacle_layer.scan.raytrace_max_range', value=7.5).to_parameter_msg(),
            Parameter(name='obstacle_layer.scan.raytrace_min_range', value=0.5).to_parameter_msg(),
            Parameter(name='obstacle_layer.scan.obstacle_max_range', value=2.5).to_parameter_msg(),
            Parameter(name='obstacle_layer.scan.obstacle_min_range', value=0.5).to_parameter_msg(),
            Parameter(name='combination_method', value = 1).to_parameter_msg(),
            Parameter(name='footprint', value = "[ [0.30, 0.30], [0.30, -0.30], [-0.30, -0.30], [-0.30, 0.30] ]" ).to_parameter_msg(),
            Parameter(name='width', value =  1).to_parameter_msg(),
            Parameter(name='height', value = 1).to_parameter_msg(),

        ]
        
        future_local_cli = self.local_costmap_cli.call_async(self.req)
        future_global_cli = self.global_costmap_cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, future_local_cli)
        rclpy.spin_until_future_complete(self, future_global_cli)

        if future_local_cli.result() is not None:
            self.get_logger().info('[future_local_cli] Request successful.')
        else:
            self.get_logger().error('[future_local_cli] Failed to set parameters.')

        if future_global_cli.result() is not None:
            self.get_logger().info('[future_global_cli] Request successful.')
        else:
            self.get_logger().error('[future_global_cli] Failed to set parameters.')


class PatrolBehaviorServer(Node):

    """
    PatrolBehaviorServer.

    PatrolBehaviorServer Class: Once you have robot_cart_laser, this class publish a 
                         reach approach with nav2, then use cmd_vel topic for final approeach
                         Finally, lift the object

    Change robot_base_frame && frame_to_follow for your own purposes

    Functions:

    Important Variables: 

    """
    def __init__(self, node_name='patrol_behavior_server_node', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)

        self.navigation_node_ = Navigation()
        self.listener_node_ = FrameListener(namespace='patrol_behavior_server')
        self.costmap_cli_node_ = costMapClients(namespace='patrol_behavior_server')


        self.declare_parameter('patrol_log_level', 'info')
        log_level = self.get_parameter('patrol_log_level').get_parameter_value().string_value
        
        self.log_levels = {
            'debug': rclpy.logging.LoggingSeverity.DEBUG,
            'info': rclpy.logging.LoggingSeverity.INFO,
            'warn':rclpy.logging.LoggingSeverity.WARN,
            'error': rclpy.logging.LoggingSeverity.ERROR,
            'fatal': rclpy.logging.LoggingSeverity.FATAL
        }

        log_level = self.log_levels.get(log_level.lower(), rclpy.logging.LoggingSeverity.INFO)
        
        self.get_logger().set_level(log_level)
        self.navigation_node_.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.listener_node_.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.stop_nav_timer = None

        #Flag to see if the process has succeed
        self.isDone = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create Server
        self.srv = self.create_service(Empty, 'patrol_behavior_server', self.service_callback)
        self.get_logger().info('patrol_behavior_server_node ready')

    def start(self):

        self.count= 0
        self.costmap_cli_node_.send_request()

        self.listener_node_.restart_var_patrol_behavior()
        self.get_logger().debug('[start]')
        self.stop_nav_timer = self.create_timer(0.7, self.stop_nav,ReentrantCallbackGroup())
        self.isDone = False

        self.listener_node_.listener_timer = self.listener_node_.create_timer(0.2, self.listener_node_.timer_function)
        # Start timer
        self.listener_node_.timer_on_ = True
        self.listener_node_.listener_mode_ = True
        self.count= 0
        # Start patrol sequence
        self.navigation_node_.start_patrol_behavior()

    def end_process(self):
        self.get_logger().debug('[end_process]')

        self.listener_node_.timer_on_ = False
        self.listener_node_.listener_mode_ = False

    def stop_nav(self):
        self.get_logger().debug('[stop_nav]')
        if self.listener_node_.frame_found_ :
            self.get_logger().info(f" [stop_nav] Stop Navigation") 
            self.navigation_node_.state_nav = 5
            self.navigation_node_.frame_found_ = True
            msg = Twist()
            msg.linear.x = 0.00
            msg.angular.z = 0.00
            self.publisher_.publish(msg)
            self.count +=1
        if(self.count >= 3):
            self.stop_nav_timer.cancel()
            

    def service_callback(self, request, response):

        self.get_logger().info('Server has been called!')
        self.start()

        while self.isDone == False:
            self.get_logger().debug('[while inside]')
            #Activar y esperar una respuesta, ya sea 5 o 6
            if self.navigation_node_.state_nav == 5 and self.navigation_node_.frame_found_:
                self.get_logger().info('[SUCCEED]')
                self.isDone = True
                self.end_process()
            if self.navigation_node_.state_nav == 6:
                self.get_logger().info('[FAILED]')
                self.isDone = True
                self.end_process()
            time.sleep(1)
        self.get_logger().info('[Outside while]')
        self.stop_nav_timer.cancel()
        return response

def main():

    rclpy.init()

    patrol_behavior_node = PatrolBehaviorServer()
    navigation_node = patrol_behavior_node.navigation_node_
    frame_listener_node = patrol_behavior_node.listener_node_
    costmap_cli_node = patrol_behavior_node.costmap_cli_node_

    executor = MultiThreadedExecutor()

    executor.add_node(patrol_behavior_node)
    executor.add_node(navigation_node)
    executor.add_node(frame_listener_node)
    executor.add_node(costmap_cli_node)

    try:
        print("### Server Patrol Behavior ###")
        executor.spin()
    except KeyboardInterrupt:
        print("[ERROR]")
        patrol_behavior_node.get_logger().info('Keyboard interrupt, shutting down.\n')
    except Exception as e:
        patrol_behavior_node.get_logger().error(f'Exception caught: {str(e)}')
    finally:
        if executor is not None:
            executor.shutdown()
        
        if patrol_behavior_node is not None:
            patrol_behavior_node.destroy_node()
        
        if navigation_node is not None:
            navigation_node.destroy_node()
        
        if frame_listener_node is not None:
            frame_listener_node.destroy_node()
        
        rclpy.shutdown()

    return None

if __name__ == '__main__':
    main()