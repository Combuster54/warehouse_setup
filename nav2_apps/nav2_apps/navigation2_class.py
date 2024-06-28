#! /usr/bin/env python3

import os
from rclpy.time import Time

"""
approach_to_frame se mantiene asi, se necesita enviar la transofmracion

"""

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
from geometry_msgs.msg import Polygon, Point32, Twist
# from attach_shelf.srv import GoToLoading
from tf2_ros import TransformListener, Buffer, TransformException
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# ----- #

from ament_index_python.packages import get_package_share_directory

###### Init Map Position  ######

request_init_position = 'init_position'

###### WAYPOINTS  ######

request_first_waypoint = 'first_waypoint'
request_second_waypoint = 'second_waypoint'
request_third_waypoint = 'third_waypoint'
request_fourth__waypoint = 'fourth_waypoint'

request_loading_position = 'loading_position'
request_shipping_position = 'shipping_position'
####################


###### Server Shelf Position ######
####################
request_init_position = 'init_position'
request_second_position = 'second_position'
request_approach_unload_position = 'approach_unload_position'
request_unload_position = 'unload_position'
####################

class Navigation(Node):

    def __init__(self, node_name='navigation_node', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)

        #buffer
        self.tf_buffer_ = Buffer()

        self.state_nav = 1
        self.navigator = BasicNavigator()
        self.rate1 = self.create_rate(10,self.get_clock())  # 10 Hz, cada 0.1 segundos
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.duration_backward = 10
        self.duration_forward = 10
        self.duration_spin = 10 
        self.frame_found_ = False

        self.patrol_points = {
        
            "first_waypoint":   [0.918871, -1.02585, 0.000,
                                -0.000, -0.000, -0.706068,  0.708144],

            "second_waypoint": [1.6073, -0.107815, 0.000,
                                -0.000, -0.000, 0.616142, 0.787635],

            "third_waypoint": [2.89698,   -0.434929, 0.000, 
                                -0.000, -0.000, 0.519157,0.854679],

            "fourth_waypoint": [3.66658,   -0.0236077, 0.000, 
                                -0.000, -0.000, -0.456118, 0.889919],  # Angle: -0.947256

        }

        self.RB1_position = {


            "init_position" :   [0.031, -0.023, -0.000,
                                -0.000, -0.000, -0.000, 1.000],

           'second_position': [2.00, -0.351, 0.025,
                                 0.000,  0.000,  -0.660, 0.752],

            'approach_unload_position': [4.326, -0.351, 0.025,
                                 0.000,  0.000,  -0.660, 0.752],

            "unload_position": [4.410, -1.650, 0.025,
                                -0.000, -0.000,  -0.747, 0.665],
        }
        
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # call the navigation process once here
        self.state_nav = 1
        #self.set_pos_init()
        #time.sleep(4)
        self.nav_timer = None
        self.poses = []
        self.approach_is_done = False
        self.nav_fails = 0
        self.get_logger().info(f'{namespace}/{node_name} ready')


        return None

    def start_patrol_behavior(self):
        self.get_logger().debug(f"[start_patrol_behavior] starting nav_timer") 
        self.frame_found_ = False
        self.state_nav = 1
        self.nav_timer = self.create_timer(10, self.patrol_behavior)

    def start_approach_shelf(self):
        self.approach_to_shelf(self.poses)


    def publish_velocity_spin(self, duration_spin, orientation):
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < Duration(seconds=duration_spin):
            msg = Twist()
            msg.linear.x = 0.00
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = orientation * 0.27
            self.publisher_.publish(msg)
            ## DEBUG
            # self.get_logger().debug(f"[publish_velocity_spin Velocity] Spin") 
            if(self.frame_found_):
                self.get_logger().debug(f"[publish_velocity_spin] break") 
                break
            self.rate1.sleep
            
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        
    def publish_velocity_backward(self):
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) <  Duration(seconds=self.duration_backward):
            msg = Twist()
            msg.linear.x = -0.15
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)

            self.get_logger().debug(f"[Publish Velocity] Backward") 
            self.rate1.sleep
            
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def publish_velocity_forward(self):
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) <   Duration(seconds=self.duration_forward):
            msg = Twist()
            msg.linear.x = 0.04
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().debug(f"[Publish Velocity] Forward") 
            self.rate1.sleep
            
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def set_pos_init(self):
        self.get_logger().debug(f"[set_pos_init]") 

        # Setting inialpose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.RB1_position[request_init_position][0]
        initial_pose.pose.position.y = self.RB1_position[request_init_position][1]
        initial_pose.pose.position.z = self.RB1_position[request_init_position][2]
        initial_pose.pose.orientation.x = self.RB1_position[request_init_position][3]
        initial_pose.pose.orientation.y = self.RB1_position[request_init_position][4]
        initial_pose.pose.orientation.z = self.RB1_position[request_init_position][5]
        initial_pose.pose.orientation.w = self.RB1_position[request_init_position][6]

        self.navigator.setInitialPose(initial_pose)

        # Wait for navigation to activate fully
        self.navigator.waitUntilNav2Active()

        return None

    def go_search_frame(self, key_dic, BT_file_name):
        self.get_logger().debug(f"[go_search_frame]") 

        shelf_item_pose = PoseStamped()
        shelf_item_pose.header.frame_id = 'map'
        shelf_item_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        shelf_item_pose.pose.position.x = self.patrol_points[key_dic][0]
        shelf_item_pose.pose.position.y = self.patrol_points[key_dic][1]
        shelf_item_pose.pose.position.z = self.patrol_points[key_dic][2]
        shelf_item_pose.pose.orientation.x = self.patrol_points[key_dic][3]
        shelf_item_pose.pose.orientation.y = self.patrol_points[key_dic][4]
        shelf_item_pose.pose.orientation.z = self.patrol_points[key_dic][5]
        shelf_item_pose.pose.orientation.w = self.patrol_points[key_dic][6]
        self.get_logger().info(f'Received request for item picking at ' + key_dic + '.') 

        # nav2_apps path to send Behavior Trees
        nav2_pkg_path = get_package_share_directory('nav2_apps')
        self.navigator.goToPose(shelf_item_pose,behavior_tree= os.path.join(nav2_pkg_path, 'behavior_trees', BT_file_name))

        # Do something during your route
        # (e.x. queue up future tasks or detect person for fine-tuned positioning)
        # Print information for workers on the robot's ETA for the demonstration
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            # if feedback and i % 5 == 0:
            #     self.get_logger().debug(f'Estimated time of arrival at ' + key_dic +
            #         ' for worker: ' + '{0:.0f}'.format(
            #             Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
            #         + ' seconds.') 

            # self.get_logger().info(f'frame_found = {self.frame_found_}') 

            if self.frame_found_ :

                result = TaskResult.SUCCEEDED
                self.get_logger().info(f'[go_search_frame] cart_frame_laser has been founded!...') 
                self.state_nav = 5 # frame found
                self.navigator.cancelTask()
                self.nav_timer.cancel()
                return None

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'(' + key_dic + ') position has been reached..!') 
            self.nav_fails = 0
            if(self.state_nav) == 4:
                self.get_logger().info(f"[state_nav] : {self.state_nav} SUCCEEDED")
                self.state_nav = 6 
            if(self.state_nav) <= 3:
                self.get_logger().info(f"[state_nav] : {self.state_nav} SUCCEEDED")
                self.state_nav += 1

        elif result == TaskResult.CANCELED:
            self.get_logger().info(f'Task at ' + key_dic  +
                ' was canceled. Returning to starting point...')
            if(self.state_nav == 4):
                self.get_logger().info("[state_nav] CANCELED")
                self.state_nav = 4

        elif result == TaskResult.FAILED:
            self.get_logger().info(f'Task at ' + key_dic + ' failed!')
            if(self.state_nav == 4):
                self.get_logger().info("[state_nav] FAILED")
                self.state_nav = 4
                self.nav_fails += 1
                if(self.nav_fails >3):
                    self.get_logger().info("[state_nav] attemps complete, goal unreachable")
                    self.nav_timer.cancel()


        while not self.navigator.isTaskComplete():
            pass
        return None

    def go_pose(self, key_dic, BT_file_name):
        self.get_logger().debug("[go_pose]")

        shelf_item_pose = PoseStamped()
        shelf_item_pose.header.frame_id = 'map'
        shelf_item_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        shelf_item_pose.pose.position.x = self.RB1_position[key_dic][0]
        shelf_item_pose.pose.position.y = self.RB1_position[key_dic][1]
        shelf_item_pose.pose.position.z = self.RB1_position[key_dic][2]
        shelf_item_pose.pose.orientation.x = self.RB1_position[key_dic][3]
        shelf_item_pose.pose.orientation.y = self.RB1_position[key_dic][4]
        shelf_item_pose.pose.orientation.z = self.RB1_position[key_dic][5]
        shelf_item_pose.pose.orientation.w = self.RB1_position[key_dic][6]
        self.get_logger().info('Received request for item picking at ' + key_dic + '.')


        # nav2_apps path to send Behavior Trees
        nav2_pkg_path = get_package_share_directory('nav2_apps')
        self.navigator.goToPose(shelf_item_pose,behavior_tree= os.path.join(nav2_pkg_path, 'behavior_trees', BT_file_name))
        
        # Do something during your route
        # (e.x. queue up future tasks or detect person for fine-tuned positioning)
        # Print information for workers on the robot's ETA for the demonstration
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(f'Estimated time of arrival at ' + key_dic +
                    ' for worker: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'(' + key_dic + ') position has been reached..!')
            if(self.state_nav == 4):
                self.get_logger().info("[state_nav] SUCCEEDED")
                self.state_nav = 5
                self.nav_timer.cancel()

        elif result == TaskResult.CANCELED:
            self.get_logger().info(f'Task at ' + key_dic  +
                ' was canceled. Returning to starting point...')
            if(self.state_nav == 4):
                self.get_logger().info("[state_nav] CANCELED")
                self.state_nav = 5
            exit(-1)

        elif result == TaskResult.FAILED:
            self.get_logger().info(f'Task at ' + key_dic + ' failed!')
            if(self.state_nav == 4):
                self.get_logger().info("[state_nav] FAILED")
                self.state_nav = 4
            exit(-1)

        while not self.navigator.isTaskComplete():
            pass
        
        return None

    def patrol_behavior(self):
        self.get_logger().debug(f"[patrol_behavior]")

        if(self.state_nav == 1):
            self.get_logger().debug(f"[START-1] state_nav = {self.state_nav}")
            self.go_search_frame(request_first_waypoint,'common_bt.xml') #which means go to this pose
            self.get_logger().info(f"[First Waypoint reach]")
            
            self.publish_velocity_spin(7,1)
            self.publish_velocity_spin(17,-1)

        if(self.state_nav ==2):
            self.get_logger().debug(f"[state_nav] {self.state_nav}")

            self.go_search_frame(request_second_waypoint,'common_bt.xml') #which means go to this pose
            self.get_logger().info(f"[Second Waypoint reach]")

            self.publish_velocity_spin(5,1)
            self.publish_velocity_spin(10,-1)

        if(self.state_nav ==3):
            self.get_logger().debug(f"[state_nav] {self.state_nav}")

            self.go_search_frame(request_third_waypoint,'common_bt.xml') #which means go to this pose
            self.get_logger().info(f"[Third Waypoint reach]")
            self.publish_velocity_spin(5,1)
            self.publish_velocity_spin(10,-1)

        if( self.state_nav == 4):
            self.get_logger().debug(f"[state_nav] {self.state_nav}")

            self.go_search_frame(request_fourth__waypoint,'common_bt.xml') #which means go to this pose
            self.get_logger().info(f"[Fourth Waypoint reach]")
            self.publish_velocity_spin(5,1)
            self.publish_velocity_spin(6,-1)

        if( self.state_nav == 5 ):
            self.get_logger().debug(f"[state_nav] {self.state_nav}")
            self.get_logger().info(f"[Shelf object was found]")
            self.nav_timer.cancel()

        if ( self.state_nav == 6):
            self.get_logger().error(f"Shelf not found")
            self.nav_timer.cancel()

    def approach_to_frame(self, goal_frame, transform):

        shelf_item_pose = PoseStamped()
        shelf_item_pose.header.frame_id = 'map'
        shelf_item_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        shelf_item_pose.pose.position.x = transform.transform.translation.x
        shelf_item_pose.pose.position.y = transform.transform.translation.y
        shelf_item_pose.pose.position.z = transform.transform.translation.z
        shelf_item_pose.pose.orientation.x = transform.transform.rotation.x
        shelf_item_pose.pose.orientation.y = transform.transform.rotation.y
        shelf_item_pose.pose.orientation.z = transform.transform.rotation.z
        shelf_item_pose.pose.orientation.w = transform.transform.rotation.w

        self.get_logger().info(f'Received request for item picking at ' + goal_frame + '.')

        # nav2_apps path to send Behavior Trees
        nav2_pkg_path = get_package_share_directory('nav2_apps')
        self.navigator.goToPose(shelf_item_pose)

        # Do something during your route
        # (e.x. queue up future tasks or detect person for fine-tuned positioning)
        # Print information for workers on the robot's ETA for the demonstration
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(f'Estimated time of arrival at ' + 'nav2_goal' +
                    ' for worker: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('(' + 'nav2_goal' + ') position has been reached..!')
            if self.state_nav == 1 :
                self.state_nav = 2
            elif self.state_nav == 2 :
                self.state_nav = 3               
            exit(-1)

        elif result == TaskResult.CANCELED:
            self.get_logger().info('Task at ' + 'nav2_goal' + 'CANCELED!')
            self.state_nav = 5
            exit(-1)

        elif result == TaskResult.FAILED:
            self.get_logger().error('Task at ' + 'nav2_goal' + 'FAILED!')
            self.state_nav = 5
            exit(-1)

        while not self.navigator.isTaskComplete():
            pass
    
        return None

    def approach_to_shelf(self, goal_poses):

        approach_point_pose = goal_poses[0]
        under_shelf_point = goal_poses[1]
        
        self.get_logger().info('Received request for item picking at ' + 'approach_point' + '.')

        # nav2_apps path to send Behavior Trees
        nav2_pkg_path = get_package_share_directory('nav2_apps')
        self.navigator.goToPose(approach_point_pose)

        # Do something during your route
        # (e.x. queue up future tasks or detect person for fine-tuned positioning)
        # Print information for workers on the robot's ETA for the demonstration
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Estimated time of arrival at ' + 'nav2_goal' +
                    ' for worker: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('(' + 'nav2_goal' + ') position has been reached..!')

            self.navigator.goToPose(under_shelf_point)
            while not self.navigator.isTaskComplete():
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    self.get_logger().info('Estimated time of arrival at ' + 'nav2_goal' +
                        ' for worker: ' + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('(' + 'nav2_goal' + ') position has been reached..!')

            self.navigator.goToPose(under_shelf_point)
            while not self.navigator.isTaskComplete():
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    self.get_logger().info('Estimated time of arrival at ' + 'nav2_goal' +
                        ' for worker: ' + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('(' + 'shelf' + ') position has been reached..!')
                exit(-1)

            elif result == TaskResult.CANCELED:
                self.get_logger().error('Task at ' + 'nav2_goal' + 'CANCELED!')
                self.state_nav = 5
                exit(-1)

            elif result == TaskResult.FAILED:
                self.get_logger().info('Task at ' + 'nav2_goal' + ' FAILED!')
                self.state_nav = 5
                exit(-1)

        elif result == TaskResult.CANCELED:
            self.get_logger().info('Task at ' + 'nav2_goal' + 'CANCELED!')
            self.state_nav = 5
            exit(-1)

        elif result == TaskResult.FAILED:
            self.get_logger().error('Task at ' + 'nav2_goal' + ' FAILED!')
            self.state_nav = 5
            exit(-1)
        while not self.navigator.isTaskComplete():
            pass
        return None

    #follow the frame for 3 seconds
    def activated_follow(self):
        i = 0
        self.get_logger().debug(f"[activated_follow]")

        while(i<3):
            self.get_logger().debug(f"follow cart_frame_laser")
            self.follow_cart_frame.timer_on = True
            i += 1
            time.sleep(1)
        self.deactivated_follow()
        self.state_nav = 1
        return None

    def deactivated_follow(self):
        self.get_logger().debug(f"[deactivated_follow]")
        self.follow_cart_frame.timer_on = False
        return None

    def start_approach_to_frame(self):
        self.get_logger().info(f"state_nav = {self.state_nav}")
        if self.state_nav == 1:
            self.approach_to_frame()
        if self.state_nav == 5:
            self.follow_cart_frame.call_timer()
            self.activated_follow()
