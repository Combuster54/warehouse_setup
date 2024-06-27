import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration

from geometry_msgs.msg import TransformStamped
import time

#Listo para usar y escuchar directamente al usar timer_on
class FrameListener(Node):

    def __init__(self, node_name='frame_listener_node', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)


        # Declare and acquire `target_frame` parameter
        self.target_frame = 'shelf_point'
        self.source_frame = 'robot_odom'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.frame_found_ = False

        # Call on_timer function every second
        self.listener_timer = self.create_timer(0.2, self.timer_function)
        
        #Activated timer
        self.timer_on_ = False
        self.t = None
        
        #Mode Listener
        self.listener_mode_ = False
        #Mode get transform
        self.get_transform_mode_ = False

        self.approach_shelf_position_transform = TransformStamped()

        self.listen_frame_1 = None
        self.listen_frame_2 = None
        self.get_logger().info(f'{namespace}/{node_name} ready')


    def restart_var_patrol_behavior(self):
        self.get_logger().debug(f'[restart_var_patrol_behavior]')
        self.listen_frame_1 = None
        self.listen_frame_2 = None
        self.listener_mode_ = False
        self.timer_on_ = False
        self.t = None
        self.get_transform_mode_ = False
        self.approach_shelf_position_transform = TransformStamped()
        self.frame_found_ = False
        self.listener_timer.cancel()

    def timer_function(self):

        if(self.timer_on_):
            self.get_logger().debug(f'[Timer on]')

            if self.listener_mode_:

                # # Store frame names in variables that will be used to
                # # compute transformations
                # target_frame = self.target_frame
                # source_frame = self.source_frame

                if self.tf_buffer.can_transform(self.source_frame, self.target_frame, rclpy.time.Time(),Duration(nanoseconds=100000)):
                    try:
                        # Obtener la transformación más reciente
                        trans = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rclpy.time.Time())

                        current_time = self.get_clock().now()
                        trans_time = rclpy.time.Time.from_msg(trans.header.stamp)

                        # Convertir tiempos a nanosegundos para la comparación
                        current_time_ns = current_time.nanoseconds
                        trans_time_ns = trans_time.nanoseconds

                        # Verificar si la transformación es reciente (usando un timeout de 1 segundo)
                        time_difference_ns = current_time_ns - trans_time_ns

                        # Convertir la diferencia a segundos
                        time_difference_s = time_difference_ns / 1e9


                        if time_difference_s < 1:
                            self.t = trans

                            if self.t.transform.translation.x == 0.0 and self.t.transform.translation.y == 0.0 and self.t.transform.translation.z == 0.0:
                                pass
                            else:
                                self.get_logger().debug(f"Frame '{self.target_frame}' is available.")
                                self.get_logger().debug(f'shelf_point founded: {self.frame_found_}')
                                self.get_logger().debug(f"Transform: {self.t}")
                                self.frame_found_ = True
                        else:
                            self.get_logger().debug(f"Transform is outdated. Current time: {current_time}, Transform time: {trans_time}")

                    except TransformException as ex:
                        self.get_logger().error(f"Could not transform {self.source_frame} to {self.target_frame}: {ex}")
                else:
                    self.get_logger().debug(f"Frame '{self.target_frame}' is not available.")
                
            if self.get_transform_mode_:

                source_frame = 'robot_base_link'
                self.get_logger().debug(f'get_transform_mode_  = True')

                if self.tf_buffer.can_transform(source_frame, self.listen_frame_1, rclpy.time.Time()):
                    self.get_logger().debug(f"Frame '{self.listen_frame_1}' is available.")
                    try:
                        self.t = self.tf_buffer.lookup_transform(
                            source_frame,
                             self.listen_frame_1,
                            rclpy.time.Time(),
                            Duration(nanoseconds=100000))


                        self.get_logger().debug(f"[get_transform_mode_] Transform: {self.t}")
                        self.get_logger().debug(f'shelf_position_transform found')
                    except TransformException as ex:
                        self.get_logger().error(f"Could not transform {self.source_frame} to {self.target_frame}: {ex}")
                else:
                    self.get_logger().debug(f"Frame '{self.listen_frame_1}' is not available.")

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()