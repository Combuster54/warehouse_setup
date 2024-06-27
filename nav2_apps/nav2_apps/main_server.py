#! /usr/bin/env python3

# ----- #
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# ----- #


from nav2_apps.cart_frame import DetectShelf
from nav2_apps.server_patrol_behavior import PatrolBehaviorServer
from nav2_apps.approach_server import ApproachServer
from nav2_apps.server_shelf_position import ShelfPositionServer

class Generic(Node):

    def __init__(self):
        super().__init__('main_node')

        self.detect_shelf_node_ = DetectShelf()
        self.patrol_behavior_node_ = PatrolBehaviorServer()
        self.approach_server_node_ = ApproachServer()
        self.shelf_position_node_ = ShelfPositionServer()


        self.detect_succed = self.create_timer(0.5, self.detect_succed_timer)
        self.count_zero_values = 0
        
        self.get_logger().info("main_node ready!")

    def detect_succed_timer(self):

        if self.detect_shelf_node_.zero_values :
            self.count_zero_values += 1
        else:
            self.count_zero_values = 0

        if self.detect_shelf_node_.zero_values >= 6:
            # resetea los valores de detect shelf
            self.get_logger().info("reseting DetectShelf values")
            self.detect_shelf_node_.frame_publisher_node_.restart_variables()
            self.detect_shelf_node_.frame_publisher_node_.listener_node_.restart_variables()


def main(args=None):
    rclpy.init(args=args)

    main_node_ = Generic()

    #DetectShelf Nodes
    detect_shelf_node = main_node_.detect_shelf_node_
    detect_shelf_frame_publisher_node_ = main_node_.detect_shelf_node_.frame_publisher_node_
    detect_shelf_frame_listener_node = main_node_.detect_shelf_node_.frame_publisher_node_.listener_node_

    #PatrolBehaviorServer Nodes
    patrol_behavior_node = main_node_.patrol_behavior_node_
    patrol_behavior_navigation_node = main_node_.patrol_behavior_node_.navigation_node_
    patrol_behavior_frame_listener_node = main_node_.patrol_behavior_node_.listener_node_

    #ApproachNodes
    approach_node = main_node_.approach_server_node_.frame_listener_node_
    approach_frame_publisher_node = main_node_.approach_server_node_.frame_publisher_node_
    approach_frame_listener_node = main_node_.approach_server_node_.frame_publisher_node_.move_shelf_node_
    approach_costmap_cli_node = main_node_.approach_server_node_.costmap_cli_node_

    #ShelfPositionServer Nodes
    shelf_position_node = main_node_.shelf_position_node_
    shelf_position_footprint_publisher_node = main_node_.shelf_position_node_.footprint_publisher_node_
    shelf_position_move_shelf_node = main_node_.shelf_position_node_.move_shelf_node_
    shelf_position_navigation_node = main_node_.shelf_position_node_.navigation_node_




    executor = MultiThreadedExecutor()
    executor.add_node(detect_shelf_node)
    executor.add_node(detect_shelf_frame_publisher_node_)
    executor.add_node(detect_shelf_frame_listener_node)

    executor.add_node(patrol_behavior_node)
    executor.add_node(patrol_behavior_navigation_node)
    executor.add_node(patrol_behavior_frame_listener_node)

    executor.add_node(approach_node)
    executor.add_node(approach_frame_publisher_node)
    executor.add_node(approach_frame_listener_node)
    executor.add_node(approach_costmap_cli_node)

    executor.add_node(shelf_position_node)
    executor.add_node(shelf_position_footprint_publisher_node)
    executor.add_node(shelf_position_move_shelf_node)
    executor.add_node(shelf_position_navigation_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        rclpy.shutdown()
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

