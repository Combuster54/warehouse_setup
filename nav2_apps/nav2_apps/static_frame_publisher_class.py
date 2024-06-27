import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time

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
            if self.tf_buffer.can_transform('robot_cart_laser', 'map', now, timeout=timeout):
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
        except TransformException as e:
            self.get_logger().warn(f'Transform error: {str(e)}')

    def set_transform(self, x, y, z):
        self.static_transform.header.stamp = self.get_clock().now().to_msg()
        self.static_transform.transform.translation.x = x
        self.static_transform.transform.translation.y = y
        self.static_transform.transform.translation.z = z

        self.static_transform.transform.rotation.x =  0.0
        self.static_transform.transform.rotation.y =  0.0
        self.static_transform.transform.rotation.z =  0.0
        self.static_transform.transform.rotation.w =  0.0

        self.broadcaster.sendTransform(self.static_transform)


class StaticFramePublisher(Node):

    def __init__(self,node_name,namespace, source_frame, target_frame,position):
        super().__init__(node_name=node_name,namespace=namespace)

        self.source_frame = source_frame
        self.target_frame = target_frame
        self.position = position

        self.static_transform = TransformStamped()
        self.static_transform.header.frame_id = self.source_frame
        self.static_transform.child_frame_id = self.target_frame

        ####
        self.static_transform.transform.translation.x = self.position.translation.x
        self.static_transform.transform.translation.y = self.position.translation.y
        self.static_transform.transform.translation.z = self.position.translation.z

        self.static_transform.transform.rotation.x = self.position.rotation.x
        self.static_transform.transform.rotation.y = self.position.rotation.y
        self.static_transform.transform.rotation.z = self.position.rotation.z
        self.static_transform.transform.rotation.w = self.position.rotation.w

        ####
        self.broadcaster = StaticTransformBroadcaster(self)

        self.timer_ = None
        ## DEBUG ##
        #self.get_logger().info('FramePublisher_Node has been started')

    def timer_on(self):
        ## DEBUG ##
        #self.get_logger().info('[timer_on]')
        self.timer_ = self.create_timer(0.2, self.publish_frame)

    def timer_off(self):
        self.timer_.cancel()

    def publish_frame(self):
        ## DEBUG ##
        #self.get_logger().info('[Try]')
        self.static_transform.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.static_transform)
        self.timer_off()
