#! /usr/bin/env python3

"""
load_shelf_point creado, para meterse de bajo y recogerlo
framepublisher.py ya publica ambos, ahora falta poder mandar la meta nav2 hacia el, hay error. 
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
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# ----- #
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Polygon, Point32, Twist, PoseStamped
# from attach_shelf.srv import GoToLoading
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster, TransformBroadcaster
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# ----- #
from std_srvs.srv import Empty, SetBool

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
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from nav2_apps.move_shelf_class import MoveShelfNode
#from nav2_apps.navigation2_class import Navigation
from nav2_apps.frame_listener_class import FrameListener

from typing import Tuple, Any

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
            Parameter(name='inflation_layer.inflation_radius', value=0.15).to_parameter_msg(),
            Parameter(name='inflation_layer.cost_scaling_factor', value = 5.0).to_parameter_msg(),
            Parameter(name='obstacle_layer.scan.raytrace_max_range', value=7.5).to_parameter_msg(),
            Parameter(name='obstacle_layer.scan.raytrace_min_range', value=2.5).to_parameter_msg(),
            Parameter(name='obstacle_layer.scan.obstacle_max_range', value=4.5).to_parameter_msg(),
            Parameter(name='obstacle_layer.scan.obstacle_min_range', value=2.5).to_parameter_msg(),
            Parameter(name='combination_method', value = 1).to_parameter_msg(),
            Parameter(name='footprint', value = "[ [0.45, 0.45], [0.45, -0.45], [-0.45, -0.45], [-0.45, 0.45] ]" ).to_parameter_msg(),
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

# probar el valor de publicaciond de approach_point y definir cuando cambiar y cuando no
class FramePublisher(Node):

    def __init__(self,node_name='frame_publisher_node',namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)

        # Initialize MoveShelfNode Class
        self.move_shelf_node_ = MoveShelfNode(namespace='approach_server')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        
        self.timer = None

        self.target_frame = 'shelf_point'
        # Transformation settings
        self.approach_transform = TransformStamped()
        self.approach_transform.header.frame_id = self.target_frame 
        self.approach_transform.child_frame_id = 'approach_point'

        self.shelf_transform = TransformStamped()
        self.shelf_transform.header.frame_id = self.target_frame 
        self.shelf_transform.child_frame_id = 'under_shelf'

        self.get_logger().info(f'{namespace}/{node_name} ready')


    def timer_on(self):
        self.timer = self.create_timer(1.1, self.publish_frame)
        self.get_logger().debug("[timer_on]")

    def timer_off(self):
        self.get_logger().debug("[timer_off]")
        if self.timer:
            self.timer.cancel()

    def publish_frame(self):
        self.get_logger().debug("[publish_frame]")
        try:
            self.get_logger().debug("[publish_frame] try ")
            self.set_approach_transform(-0.7, 0.0, 0.0)
            # self.set_shelf_transform(0.3,0.0,0.0)

            self.broadcaster.sendTransform(self.approach_transform)
            # self.broadcaster.sendTransform(self.shelf_transform)

        except TransformException as e:
            self.get_logger().warn(f'Transform error: {str(e)}')
            self.timer_off()

    def set_approach_transform(self, x, y, z):
        self.get_logger().debug(f'[set_approach_transform]')

        self.approach_transform.header.stamp = self.get_clock().now().to_msg()
        self.approach_transform.transform.translation.x = x
        self.approach_transform.transform.translation.y = y
        self.approach_transform.transform.translation.z = z
        self.approach_transform.transform.rotation.x =  0.0
        self.approach_transform.transform.rotation.y =  0.0
        self.approach_transform.transform.rotation.z =  0.0
        self.approach_transform.transform.rotation.w =  1.0

    def set_shelf_transform(self, x, y, z):
        self.get_logger().debug(f'[set_approach_transform]')

        self.shelf_transform.header.stamp = self.get_clock().now().to_msg()
        self.shelf_transform.transform.translation.x = x
        self.shelf_transform.transform.translation.y = y
        self.shelf_transform.transform.translation.z = z
        self.shelf_transform.transform.rotation.x =  0.0
        self.shelf_transform.transform.rotation.y =  0.0
        self.shelf_transform.transform.rotation.z =  0.0
        self.shelf_transform.transform.rotation.w =  1.0


class ApproachServer(Node):

    """
    ApproachServer.

    ApproachServer Class: Once you have shelf_point, this class publish a 
                         frame to have a better approach by using
                         cmd_vel topic for final approeach.
                         Finally, lift the object

    Functions:

    Important Variables: 

    """
    def __init__(self):
        super().__init__('approach_server_node')

        #Flag to see if the process has succeed
        self.isDone = False

        # Initialize FramePublisher Class
        self.frame_publisher_node_ = FramePublisher(namespace='approach_server')
        #Initialize FrameListener Class 
        self.frame_listener_node_ = FrameListener(namespace='approach_server') #cart_laser_frame && robot_odom by default

        self.costmap_cli_node_ = costMapClients(namespace='approach_server')

        self.declare_parameter('approach_log_level', 'info')
        log_level = self.get_parameter('approach_log_level').get_parameter_value().string_value
        

        self.log_levels = {
            'debug': rclpy.logging.LoggingSeverity.DEBUG,
            'info': rclpy.logging.LoggingSeverity.INFO,
            'warn':rclpy.logging.LoggingSeverity.WARN,
            'error': rclpy.logging.LoggingSeverity.ERROR,
            'fatal': rclpy.logging.LoggingSeverity.FATAL
        }

        log_level = self.log_levels.get(log_level.lower(), rclpy.logging.LoggingSeverity.INFO)

        self.get_logger().set_level(log_level)
        self.frame_publisher_node_.get_logger().set_level(log_level)
        self.frame_listener_node_.get_logger().set_level(log_level)




        self.frame_listener_node_.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        # Create Publisher
        self.cmd_vel_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.cmd_vel_pub_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)


        self.yaw = 0.0
        self.x_error = 0.0
        self.y_error = 0.0

        self.error_yaw = 0.0 
        self.distance = None
        self.approach_dist = 0.040

        self.duration_forward = Duration(nanoseconds=6.7e+9)

        #Flags

        self.detect_shelf_point_flag = False
        self.first_point_flag = False
        self.inside_shelf_flag = False
        self.align_with_shelf = False
        # Create Server
        self.srv = self.create_service(SetBool, 'approach_shelf_server', self.service_callback)
        self.get_logger().info(f'approach_server_node ready')

    def get_transform(self, parent_frame: str , child_frame: str)->  Tuple[PoseStamped, Any]:
        
        assert isinstance(parent_frame, str), 'Strings only!'
        assert isinstance(child_frame, str),  'Strings only!'
        
        translation = self.frame_listener_node_.t.transform.translation
        rotation   = self.frame_listener_node_.t.transform.rotation
        self.get_logger().debug(f"[traslation] : {translation} rotation : {rotation} ")

        return translation , rotation

    def updatePose(self, parent_frame: str , child_frame: str ):
        self.get_logger().debug(f"[updatePose]")

        translation, rotation = self.get_transform( parent_frame, child_frame)
        self.x_error = translation.x
        self.y_error = translation.y

        self.get_logger().debug(f"Traslation {translation} rotation {rotation}")
        self.error_yaw =  math.atan2(self.x_error, self.y_error)
        roll, pitch, self.yaw = self.euler_from_quaternion(rotation.x, rotation.y, rotation.z , rotation.w)

    def followFrame(self, parent_frame: str , child_frame: str):
        
            loop_rate = self.create_rate(10)
            self.get_logger().debug(f"[followFrame]")

            while (1):
                velocity = Twist()
                self.updatePose(parent_frame, child_frame)

                self.distance = math.sqrt(self.x_error**2 + self.y_error**2)
 
                self.get_logger().info(f'[followFrame] distance {self.distance}')

                self.error_yaw = math.pi / 2 - math.atan2(self.x_error,  self.y_error) 
                abs_error_yaw = abs(self.error_yaw)
                self.get_logger().info(f'[followFrame] error_yaw {abs_error_yaw}')

                #Approach with focus in point
                if abs_error_yaw > math.pi / 90:
                    velocity.angular.z = 0.15 if self.error_yaw > 0 else -0.15
                else:
                    velocity.linear.x = 0.1
                    velocity.angular.z = 0.0

                # if self.error_yaw >= 0.0:
                #     if  self.y_error >= 0
                #         velocity.angular.z = 0.10
                #     else:
                #         velocity.angular.z = -1.0*  0.10

                if self.distance <= self.approach_dist:
                    velocity.linear.x = 0.0
                    velocity.angular.z = 0.0
                    return True

                self.cmd_vel_pub_.publish(velocity)
                loop_rate.sleep()

    def stop(self):
        velocity = Twist()
        velocity.linear.x  = 0.0
        velocity.angular.z = 0.0
        self.cmd_vel_pub_.publish(velocity)
        self.get_logger().debug('Stop!')


    def align_to_shelf(self, parent_frame: str , child_frame: str, turn_speed: float):
        self.get_logger().debug(f'[align_to_shelf]')

        loop_rate = self.create_rate(10)
        velocity = Twist()

        while abs(self.yaw) > 0.01:

            self.get_logger().info(f'[align_to_shelf] yaw_error = {self.yaw}')
            # Actualizar variables
            self.updatePose(parent_frame, child_frame)

            if self.yaw >= 0.0:
                velocity.angular.z = turn_speed
            else:
                velocity.angular.z = -1.0 * turn_speed

            self.cmd_vel_pub_.publish(velocity)
            loop_rate.sleep()

        self.align_with_shelf = True

    def euler_from_quaternion(self, x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians

    def publish_velocity_forward(self):
        start_time = self.get_clock().now()
        loop_rate = self.create_rate(10)
        self.get_logger().debug(f'[publish_velocity_forward]')

        while rclpy.ok() and (self.get_clock().now() - start_time) < self.duration_forward:
            msg = Twist()
            msg.linear.x = 0.15
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub_.publish(msg)
            self.get_logger().debug(f'[publish_velocity_forward] Forward')
            
            loop_rate.sleep()
            
        msg.linear.x = 0.0
        self.cmd_vel_pub_.publish(msg)       

    def service_callback(self, request, response):

        self.get_logger().debug('Server has been called!')

        # Start timer
        self.frame_listener_node_.timer_on_ = True
        self.frame_listener_node_.listener_mode_ = True

        count = 0
        while count <= 3:
            time.sleep(1)
            count += 1

        if self.frame_listener_node_.frame_found_:

            #start flags
            self.first_point_flag = False
            self.second_point_flag = False

            self.detect_shelf_point_flag = True
            self.frame_listener_node_.listener_mode_ = False
            #founded, start 
            self.get_logger().debug('frame_found_ true')
            self.frame_publisher_node_.timer_on()

            time.sleep(1)

            self.frame_listener_node_.get_transform_mode_ = True
            self.frame_listener_node_.listen_frame_1 = "approach_point"
            
            time.sleep(1)

            #First point
            self.first_point_flag = self.followFrame('robot_base_link','approach_point')
            self.stop()

            print(self.first_point_flag)
            #Comprobar si se alcanzo el frame
            if self.first_point_flag == False:
                self.get_logger().debug('approach_point failed')
                response.success = False
                response.message = "approach_point failed"
                return response
            
            self.get_logger().debug('approach_point reached')
            self.align_to_shelf('robot_base_link','approach_point', 0.05)
            self.stop()

            if self.align_with_shelf == False:
                response.success = False
                response.message = "align_with_shelf failed"
                self.get_logger().debug('align_with_shelf failed')
                return response
            self.get_logger().debug('align_with_shelf reached')

            time.sleep(3)
            self.stop()
            self.publish_velocity_forward()
            #Lifting shelf
            self.frame_publisher_node_.move_shelf_node_.publish_message_up()

            self.costmap_cli_node_.send_request()
            
            if self.detect_shelf_point_flag and self.first_point_flag and self.align_with_shelf :

                self.get_logger().info('aprroach shelf_srv complete each flag')
                self.isDone = True
                response.success = True
                response.message = "Aprroach shelf_srv complete each flag"
            else:
                self.get_logger().info('aprroach shelf_srv error, one flag is false')
                self.get_logger().info(f'detect_shelf_point_flag = {self.detect_shelf_point_flag}')
                self.get_logger().info(f'first_point_flag = {self.first_point_flag}')
                self.get_logger().info(f'align_with_shelf = {self.align_with_shelf}')
                response.success = False

            return response

        else:
            self.get_logger().warn("Shelf wasn't detect")
            self.isDone = False

        return response

def main():

    rclpy.init()

    approach_server_node = ApproachServer()

    frame_listener_node =  approach_server_node.frame_listener_node_
    frame_publisher_node =  approach_server_node.frame_publisher_node_
    move_shelf_node =  approach_server_node.frame_publisher_node_.move_shelf_node_
    costmap_cli_node = approach_server_node.costmap_cli_node_


    executor = MultiThreadedExecutor()

    executor.add_node(approach_server_node)
    executor.add_node(move_shelf_node)
    executor.add_node(costmap_cli_node)
    executor.add_node(frame_publisher_node)
    executor.add_node(frame_listener_node)

    try:
        print("### MoveShelf Initialize ###")
        executor.spin()#Where I put the spin? o I just use while instead?
    except KeyboardInterrupt :
        #Is it possible to have a general get_logger? how?
        print("[ERROR]")
        approach_server_node.get_logger().debug('Keyboard interrupt, shutting down.\n')

    #There are a lot of nodes!
    approach_server_node.destroy_node()
    frame_listener_node.destroy_node()
    frame_publisher_node.destroy_node()
    costmap_cli_node.destroy_node()
    
    rclpy.shutdown()

    return None

if __name__ == '__main__':
    main()