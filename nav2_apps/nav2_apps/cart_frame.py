import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import StaticTransformBroadcaster, TransformListener, Buffer, TransformBroadcaster ,TransformException
import math
import time

import numpy as np

from scipy.spatial.transform import Rotation

#El parametro que se va a actualizra constantemente para que esto funcione es la transformada
class FrameListener(Node):

    def __init__(self,node_name='frame_listener_node',namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)

        # Declare and acquire `target_frame` parameter
        self.target_frame = 'robot_front_laser_base_link'
        self.source_frame = 'robot_odom'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Call on_timer function every second
        self.timer = self.create_timer(1.0/60, self.timer)

        self.t = None
        self.timer_on_ = True
        self.linear_matrix_ = np.eye(4)
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        self.tx = 0.0
        self.ty = 0.0 
        self.tz = 0.0 

        self.tx_matrix = 0.0
        self.ty_matrix = 0.0
        self.tz_matrix = 0.0

        self.get_logger().info(f'{namespace}/{node_name}is ready')


    def restart_variables(self):
        self.t = None
        self.timer_on_ = True
        self.linear_matrix_ = np.eye(4)
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        self.tx = 0.0
        self.ty = 0.0 
        self.tz = 0.0 

        self.tx_matrix = 0.0
        self.ty_matrix = 0.0
        self.tz_matrix = 0.0



    def timer(self):

        self.get_logger().debug(f'Timer')            
        if(self.timer_on_):
            self.get_logger().debug(f'[Timer on]')            
            target_frame = self.target_frame
            source_frame = self.source_frame

            # Look up for the transformation between target_frame and turtle2 frames
            # and send velocity commands for turtle2 to reach target_frame
            try:
                self.t = self.tf_buffer.lookup_transform(
                    source_frame,
                    target_frame,
                    rclpy.time.Time())

                self.tx, self.ty, self.tz = self.t.transform.translation.x , self.t.transform.translation.y, self.t.transform.translation.z
                self.qx, self.qy, self.qz, self.qw = self.t.transform.rotation.x, self.t.transform.rotation.y, self.t.transform.rotation.z, self.t.transform.rotation.w


            except TransformException as ex:
                #self.get_logger().debug(f'Could not transform {source_frame} to {target_frame}: {ex}')
                pass

class FramePublisher(Node):

    def __init__(self,node_name='frame_publisher_node',namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)


        self.listener_node_ = FrameListener(namespace='detect_shelf')

        self.tf_buffer = Buffer()
        self.aux_broadcaster = TransformBroadcaster(self)

        self.aux_transform = TransformStamped()
        self.aux_transform.header.frame_id = 'robot_odom'
        self.aux_transform.child_frame_id = 'shelf_point'
        self.on_ = False
        self.dt = 0.0
        self.theta_total = 0.0
        self.theta_final = 0.0
        time.sleep(3)
        self.T = None
        self.count = 0  
        self.frame_publisher_timer = self.create_timer(1.0 / 60, self.publish_frame)
        self.get_logger().info(f'{namespace}/{node_name}is ready')


    def cancel_publish_timer(self):
        self.frame_publisher_timer.cancel()
        self.frame_publisher_timer = None


    def restart_variables(self):
        self.aux_transform = TransformStamped()
        self.on_ = False
        self.dt = 0.0
        self.theta_total = 0.0
        self.theta_final = 0.0
        self.T = None
        self.count = 0  


    def euler_from_quaternion(self,qx,qy,qz,qw):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = qx
        y = qy
        z = qz
        w = qw

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_euler(self,roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4

        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr

        return q

    def publish_frame(self):

        # if self.on_ and self.count <=1:
        if self.on_:

            self.count +=1

            roll, pitch, yaw = self.euler_from_quaternion(self.listener_node_.qx,self.listener_node_.qy,self.listener_node_.qz,self.listener_node_.qw)

            quat = self.quaternion_from_euler(roll, pitch, yaw - self.theta_final)

            self.aux_transform.header.stamp = self.get_clock().now().to_msg()
            self.aux_transform.transform.translation.x = self.listener_node_.tx + self.dt * math.sin(self.theta_total) * math.sin(yaw) +  self.dt * math.cos(self.theta_total) * math.cos(yaw)
            self.aux_transform.transform.translation.y = self.listener_node_.ty + self.dt * math.cos(self.theta_total) * math.sin(yaw) -  self.dt * math.sin(self.theta_total) * math.cos(yaw)

            self.aux_transform.transform.translation.z = self.listener_node_.tz
            self.aux_transform.transform.rotation.x = quat[0]
            self.aux_transform.transform.rotation.y =  quat[1]
            self.aux_transform.transform.rotation.z =  quat[2]
            self.aux_transform.transform.rotation.w =  quat[3]
            self.aux_broadcaster = TransformBroadcaster(self)

            if(self.aux_transform.transform.translation.x == 0.0 and self.aux_transform.transform.translation.y == 0.0  and self.aux_transform.transform.translation.z  == 0.0 ):            
                pass
            else:
                try: 
                    self.aux_broadcaster.sendTransform(self.aux_transform)
                except AttributeError:
                    pass 
                    
class DetectShelf(Node):
    def __init__(self):
        super().__init__('detect_shelf_node')

        self.declare_parameter('log_level', 'info')
        self.declare_parameter('shelf_real_distance_arg', False)
        log_level = self.get_parameter('log_level').get_parameter_value().string_value

        self.shelf_real_distance = self.get_parameter('shelf_real_distance_arg').get_parameter_value().bool_value
        log_levels = {
            'debug': rclpy.logging.LoggingSeverity.DEBUG,
            'info': rclpy.logging.LoggingSeverity.INFO,
            'warn':rclpy.logging.LoggingSeverity.WARN,
            'error': rclpy.logging.LoggingSeverity.ERROR,
            'fatal': rclpy.logging.LoggingSeverity.FATAL
        }

        log_level = log_levels.get(log_level.lower(), rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)

        # self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.frame_publisher_node_ = FramePublisher(namespace='detect_shelf')
        self.frame_publisher_node_.listener_node_.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.frame_publisher_node_.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.min_shelf_dist = None
        self.max_shelf_dist = None


        if(self.shelf_real_distance):

            self.min_shelf_dist = 0.55
            self.max_shelf_dist = 0.62
        else:
            self.min_shelf_dist = 0.65
            self.max_shelf_dist = 0.74    

        self.subscription = self.create_subscription(
                                    LaserScan,
                                    'scan',
                                    self.scan_callback,
                                    10)

        self.intensities_above_threshold = []
        self.changes = 0
        self.index_intensities = []
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        #time.sleep(5)
        self.threshold = 3400
        #self.service_callback()

        self.zero_values = True

        self.get_logger().info('detect_shelf_node ready')

    def scan_callback(self, msg):
        self.intensities_above_threshold = msg.intensities
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.changes = 0 
        self.index_intensities = []

        angle1_final = 0.0
        angle2_final = 0.0
        d1_final = 0.0
        d2_final = 0.0

        flag_up = False
        flag_down = False
        count = 0

        for i, item in enumerate(self.intensities_above_threshold):
            if item < 3000 and not flag_up and not flag_down:
                flag_up = True
                flag_down = False
            elif flag_up and item > 3000:
                if i == len(self.intensities_above_threshold) - 1:
                    self.index_intensities.append(i)
                    count += 1
                    flag_up = False
                    flag_down = False
                self.index_intensities.append(i)
                flag_down = True
                flag_up = False
            elif flag_down and item < 3000:
                count += 1
                self.index_intensities.append(i)
                flag_up = True
                flag_down = False
            elif item > 3000 and not flag_up and not flag_down:
                flag_up = False
                flag_down = True

        self.get_logger().debug(f'count : {count}')

        for i in self.index_intensities:
            self.get_logger().debug(f' intensities  {self.intensities_above_threshold[i]}')

        if count == 2 and len(self.index_intensities) == 4:

            self.get_logger().debug('[c2] count = 2')
            self.get_logger().debug(f'[c2]index_intensities {self.index_intensities}')
            self.get_logger().debug(f'[c2] len index_intensities {len(self.index_intensities)}')

            index_d1 = (self.index_intensities[0] + self.index_intensities[1]) // 2
            index_d2 = (self.index_intensities[2] + self.index_intensities[3]) // 2

            d1 = self.ranges[index_d1]
            d2 = self.ranges[index_d2]

            angle1 = self.angle_min + index_d1 * self.angle_increment
            angle2 = self.angle_min + index_d2 * self.angle_increment

            #distance between legs
            point_distance_1 =  math.sqrt(d2**2+ d1**2 - 2*d2*d1*math.cos(abs(angle2-angle1)))
            self.get_logger().debug(f'[c2 ] point_distance = {point_distance_1}')
            

            if point_distance_1 >= self.min_shelf_dist and point_distance_1 <= self.max_shelf_dist:
                
                self.get_logger().debug(f'[c2] point_distance {point_distance_1}')
                self.get_logger().debug(f'angle1 = {angle1}')
                self.get_logger().debug(f'angle2 = {angle2}')
                self.get_logger().debug(f'd1  = {d1}')
                self.get_logger().debug(f'd2  = {d2}')

                angle1_final = angle1
                angle2_final = angle2
                d1_final = d1
                d2_final = d2

                angle1_final = angle1
                angle2_final = angle2
                d1_final = d1
                d2_final = d2

        if count == 3 and len(self.index_intensities) <=7 and len(self.index_intensities) >= 5:

            self.get_logger().debug('[c3] count = 3')
            self.get_logger().debug(f'[c3] index_intensities {self.index_intensities}')
            self.get_logger().debug(f'[c3] len index_intensities {len(self.index_intensities)}')

            for i in self.index_intensities:
                self.get_logger().debug(f' distances  {self.intensities_above_threshold[i]}')

            if len(self.index_intensities) == 5:
                self.get_logger().debug(f' index_intensities = 5')

                index_d1 = (self.index_intensities[0] + self.index_intensities[1]) // 2        
                index_d2 = (self.index_intensities[2] + self.index_intensities[3]) // 2
                index_d3 = 0.0
                index_d4 = 0.0

                d1 = self.ranges[index_d1]
                d2 = self.ranges[index_d2]
                d3 = 0.0
                d4 = 0.0

            if len(self.index_intensities) >= 6:
                self.get_logger().debug(f' index_intensities = 6')

                index_d1 = (self.index_intensities[0] + self.index_intensities[1]) // 2        
                index_d2 = (self.index_intensities[2] + self.index_intensities[3]) // 2
                index_d3 = (self.index_intensities[2] + self.index_intensities[3]) // 2
                index_d4 = (self.index_intensities[4] + self.index_intensities[5]) // 2   

                d1 = self.ranges[index_d1]
                d2 = self.ranges[index_d2]
                d3 = self.ranges[index_d3]
                d4 = self.ranges[index_d4]


            self.get_logger().debug(f'd1  = {d1}')
            self.get_logger().debug(f'd2  = {d2}')
            self.get_logger().debug(f'd3  = {d3}')
            self.get_logger().debug(f'd4  = {d4}')

            angle1 = self.angle_min + index_d1 * self.angle_increment
            angle2 = self.angle_min + index_d2 * self.angle_increment
            angle3 = self.angle_min + index_d3 * self.angle_increment
            angle4 = self.angle_min + index_d4 * self.angle_increment

            self.get_logger().debug(f'angle1 = {angle1}')
            self.get_logger().debug(f'angle2 = {angle2}')
            self.get_logger().debug(f'angle3 = {angle3}')
            self.get_logger().debug(f'angle4 = {angle4}')

            point_distance_1 =  math.sqrt(d2**2+ d1**2 - 2*d2*d1*math.cos(abs(angle2-angle1)))
            self.get_logger().debug(f'point_distance_1 = {point_distance_1}')

            point_distance_2 =  math.sqrt(d4**2+ d3**2 - 2*d4*d3*math.cos(abs(angle4-angle3)))
            self.get_logger().debug(f'point_distance_2 = {point_distance_2}')

            if point_distance_1 >= self.min_shelf_dist and point_distance_1 <= self.max_shelf_dist:

                self.get_logger().debug(f'[c3] first point = {point_distance_1}')
                self.get_logger().debug(f'angle1 = {angle1}')
                self.get_logger().debug(f'angle2 = {angle2}')
                self.get_logger().debug(f'd1  = {d1}')
                self.get_logger().debug(f'd2  = {d2}')

                angle1_final = angle1
                angle2_final = angle2
                d1_final = d1
                d2_final = d2
                
            if point_distance_2 >= self.min_shelf_dist and point_distance_2 <= self.max_shelf_dist:

                self.get_logger().debug(f'[c3] second point = {point_distance_2}')
                self.get_logger().debug(f'angle3 = {angle3}')
                self.get_logger().debug(f'angle4 = {angle4}')
                self.get_logger().debug(f'd3  = {d3}')
                self.get_logger().debug(f'd4  = {d4}')

                angle1_final = angle3
                angle2_final = angle4
                d1_final = d3
                d2_final = d4

        if count == 4 and len(self.index_intensities) == 8:
            self.get_logger().debug('[c4] count =4')
            self.get_logger().debug(f'[c4] index_intensities {self.index_intensities}')
            self.get_logger().debug(f'[c4] len index_intensities {len(self.index_intensities)}')
            index_d1 = (self.index_intensities[0] + self.index_intensities[1]) // 2
            index_d2 = (self.index_intensities[2] + self.index_intensities[3]) // 2

            index_d3 = (self.index_intensities[4] + self.index_intensities[5]) // 2
            index_d4 = (self.index_intensities[6] + self.index_intensities[7]) // 2

            d1 = self.ranges[index_d1]
            d2 = self.ranges[index_d2]

            d3 = self.ranges[index_d3]
            d4 = self.ranges[index_d4]

            angle1 = self.angle_min + index_d1 * self.angle_increment
            angle2 = self.angle_min + index_d2 * self.angle_increment
            angle3 = self.angle_min + index_d3 * self.angle_increment
            angle4 = self.angle_min + index_d4 * self.angle_increment

            self.get_logger().debug(f'angle 1 = {angle1}')
            self.get_logger().debug(f'angle 2 = {angle2}')
            self.get_logger().debug(f'angle 3 = {angle3}')
            self.get_logger().debug(f'angle 4 = {angle4}')

            point_distance_1 =  math.sqrt(d2**2+ d1**2 - 2*d2*d1*math.cos(abs(angle2-angle1)))
            self.get_logger().debug(f'shelf = {point_distance_1}')
            
            point_distance_2 =  math.sqrt(d4**2+ d3**2 - 2*d4*d3*math.cos(abs(angle4-angle3)))
            self.get_logger().debug(f'red_thing = {point_distance_2}')

            if point_distance_1 >= self.min_shelf_dist and point_distance_1 <= self.max_shelf_dist:

                self.get_logger().debug(f'[c4] first point = {point_distance_1}')
                self.get_logger().debug(f'angle1 = {angle1}')
                self.get_logger().debug(f'angle2 = {angle2}')
                self.get_logger().debug(f'd1  = {d1}')
                self.get_logger().debug(f'd2  = {d2}')

                angle1_final = angle1
                angle2_final = angle2
                d1_final = d1
                d2_final = d2
            
            elif point_distance_2 >= self.min_shelf_dist and point_distance_2 <= self.max_shelf_dist:

                self.get_logger().debug(f'[c4] second point = {point_distance_1}')
                self.get_logger().debug(f'angle 3 = {angle3}')
                self.get_logger().debug(f'angle 4 = {angle4}')
                self.get_logger().debug(f'd3  = {d3}')
                self.get_logger().debug(f'd4  = {d4}')

                angle1_final = angle3
                angle2_final = angle4
                d1_final = d3
                d2_final = d4


        if angle1_final!= 0.0 and angle2_final != 0.0 and d1_final != 0.0 and d2_final != 0.0:

            p1_i = d1_final*math.cos(angle1_final)
            p1_j = d1_final*math.sin(angle1_final)

            p2_i = d2_final*math.cos(angle2_final)
            p2_j = d2_final*math.sin(angle2_final)

            dt_alfa = p1_i + p2_i
            dt_beta = p1_j + p2_j
            
            distance = math.sqrt(dt_alfa**2 + dt_beta**2)
            dt = distance/2

            theta_total = math.atan2(dt_beta,dt_alfa)
            
            self.get_logger().debug(f'angle2_final {angle2_final}')
            self.get_logger().debug(f'theta_total {theta_total}')
            self.get_logger().debug(f'd2 {d2_final}')
            self.get_logger().debug(f'dt {dt}')
            self.get_logger().debug(f'abs(theta_total- angle2_final ) {abs(theta_total- angle2_final )}')
            expr = d2_final**2 + dt**2 - 2 * d2_final * dt * math.cos(abs(theta_total - angle2_final))
            self.get_logger().debug(f'expr  {expr}')
            c = math.sqrt(d2_final**2+ dt**2 - 2*d2_final*dt*math.cos(abs(theta_total- angle2_final )))
            self.get_logger().debug(f'c  {c}')

            try:
                alpha_theta =  abs(math.acos(( c**2 + dt**2 - d2_final**2)/(2 * c * dt) ))
                theta_final = math.pi /2 + theta_total - alpha_theta

                self.get_logger().debug(f'alpha_theta = {alpha_theta}')

                theta_final = math.pi /2 + theta_total - alpha_theta

                self.get_logger().debug(f'dt {dt}')
                self.get_logger().debug(f'theta_total {theta_total}')
                self.get_logger().debug(f'theta_final {theta_final}')

                self.frame_publisher_node_.count = 0.0
                self.frame_publisher_node_.dt = dt
                self.frame_publisher_node_.theta_total = theta_total
                self.frame_publisher_node_.theta_final = theta_final
                self.frame_publisher_node_.on_ = True
                self.frame_publisher_node_.listener_node_.timer_on_= True
            except ZeroDivisionError:
                self.frame_publisher_node_.on_ = False
                self.frame_publisher_node_.aux_broadcaster = None
                self.frame_publisher_node_.listener_node_.timer_on_= False

                self.get_logger().error(f'ZeroDivisionError {e}')
            except ValueError: 
                self.frame_publisher_node_.on_ = False
                self.frame_publisher_node_.aux_broadcaster = None
                self.frame_publisher_node_.listener_node_.timer_on_= False

                self.get_logger().error(f'ValueError {e}')
            except Exception  as e:
                self.frame_publisher_node_.on_ = False
                self.frame_publisher_node_.aux_broadcaster = None
                self.frame_publisher_node_.listener_node_.timer_on_= False
                self.get_logger().error(f'Error {e}')

        else:
            self.get_logger().debug(f'No values')

            self.zero_values = True
            self.frame_publisher_node_.on_ = False
            self.frame_publisher_node_.aux_broadcaster = None
            self.frame_publisher_node_.listener_node_.timer_on_= False

        # loop_rate.sleep()

def main(args=None):
    rclpy.init(args=args)

    app_server_node_ = DetectShelf()
    frame_publisher_node_ = app_server_node_.frame_publisher_node_
    frame_listener_node_ = app_server_node_.frame_publisher_node_.listener_node_

    executor = MultiThreadedExecutor()
    executor.add_node(app_server_node_)
    executor.add_node(frame_publisher_node_)
    executor.add_node(frame_listener_node_)

    try:
        executor.spin()
    except KeyboardInterrupt:
        rclpy.shutdown()
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
