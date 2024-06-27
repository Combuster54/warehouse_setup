import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

from rclpy.duration import Duration
import time

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from threading import Thread
from concurrent.futures import ThreadPoolExecutor


class AmclSub(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.callback_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10,
            callback_group=self.callback_group)
        self.msg = PoseWithCovarianceStamped()
        self.get_logger().info('amcl subscriber on')

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        # self.get_logger().info(" A ")

        self.msg = msg

class AmclClient(Node):

    def __init__(self):
        super().__init__('amcl_client_node')

        self.client_cb_group = ReentrantCallbackGroup()
        self.cli = self.create_client(Empty, '/reinitialize_global_localization', callback_group=self.client_cb_group)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Empty.Request()
        self.send_request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class SelfLocalizeRB1(Node):
    
    def __init__(self):
        super().__init__('self_localize_rb1_node')
        # Init Publisher
        # self.publisher_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.amcl_sub_node = AmclSub()
        self.amcl_client_node = AmclClient()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd = Twist()
        self.sub_msg = PoseWithCovarianceStamped()
        # Initialize Service Client
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()

        # Other stuff
        self.ctrl_c = False
        self.rate = self.create_rate(10)

        self.req = Empty.Request()

        self.forward_count  = 0.0
        self.rotation_count = 0.0

        #Start timer
        self.call_srv_flag    = False
        self.move_square = False
        self.count_squared = 0.0
        self.wait_srv = True
        self.timer_ = self.create_timer(0.5, self.start, callback_group=self.timer_cb_group)

        self.get_logger().info("Node initialized")

        time.sleep(3)
        self.service_available = True

        # self.executor_pool = ThreadPoolExecutor(max_workers=3)


    def shutdownhook(self):
        # works better than the rclpy.is_shutdown()
        self.stop_husky()
        self.ctrl_c = True

    def publish_velocity_spin(self):

        msg = Twist()
        msg.linear.x = 0.00
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.30
        self.publisher_.publish(msg)

    def publish_linear_velocity(self):
            msg = Twist()
            msg.linear.x =  0.20
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)

    def calculate_covariance(self):
        self.sub_msg = self.amcl_sub_node.msg

        # self.get_logger().info("######## Calculating Covariance...")
        # self.get_logger().info(f"{self.sub_msg.pose}")

        cov_x = self.sub_msg.pose.covariance[0]
        cov_y = self.sub_msg.pose.covariance[7]
        cov_z = self.sub_msg.pose.covariance[35]
        # self.get_logger().info("## Cov X: " + str(cov_x) + " ## Cov Y: " + str(cov_y) + " ## Cov Z: " + str(cov_z))
        cov = (cov_x+cov_y+cov_z)/3
        self.get_logger().info(f"######## Covariance = {cov}")

        return cov

    def start(self):
        ## Debug
        #self.get_logger().info("########")     
        try:
            if self.service_available:
                self.move_square = True
                
            if self.count_squared >= 106.0:
                self.move_square = False

            if self.move_square:
                #self.get_logger().info(f"[move_square] {self.count_squared} ")
                self.count_squared +=1
                if self.count_squared >= 104.0:
                    self.move_square = False

                if self.forward_count <=10:
                    self.publish_linear_velocity()
                    self.forward_count += 1 
                elif self.rotation_count <= 14:
                    self.publish_velocity_spin()
                    self.rotation_count += 1 
                else:
                    self.forward_count = 0.0
                    self.rotation_count = 0.0

            if (self.count_squared >= 104.0 and self.move_square == False):
                    
                cov = 1
                cov = self.calculate_covariance()

                if cov > 0.65:
                    self.get_logger().info("######## Total Covariance is greater than 0.65. Repeating the process...")
                    self.amcl_client_node.send_request()
                    self.count_squared = 0.0

                else:
                    self.get_logger().info("######## Total Covariance is lower than 0.65. Robot correctly localized!")
                    self.get_logger().info("######## Timer canceled")
                    self.timer_.cancel()

        except Exception as e:
            self.get_logger().error(f"Error in start function: {e}")

def spin_srv(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass

def main(args=None):
    rclpy.init(args=args)


    move_rb1_node_ = SelfLocalizeRB1()

    executor = MultiThreadedExecutor()
    amcl_sub_node_ = move_rb1_node_.amcl_sub_node
    amcl_client_node_ = move_rb1_node_.amcl_client_node
    # amcl_sub_executor = SingleThreadedExecutor()
    # amcl_sub_thread = Thread(target=spin_srv, args=(amcl_sub_executor, ), daemon=True)
    # amcl_sub_thread.start()

    executor = MultiThreadedExecutor()
    executor.add_node(move_rb1_node_)
    executor.add_node(amcl_sub_node_)
    executor.add_node(amcl_client_node_)

    try:
        move_rb1_node_.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        move_rb1_node_.get_logger().info('Keyboard interrupt, shutting down.\n')

    move_rb1_node_.destroy_node()
    amcl_sub_node_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()