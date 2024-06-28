
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('my_node')

        self.local_costmap_cli = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        self.global_costmap_cli = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')

        while not self.local_costmap_cli.wait_for_service(timeout_sec=1.0) and not  self.global_costmap_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('services not available yet, waiting again...')
        self.req = SetParameters.Request()
        self.get_logger().info('Client call succeeded')

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
            Parameter(name='combination_method', value = 0).to_parameter_msg(),
            Parameter(name='footprint', value = "[ [0.45, 0.45], [0.45, -0.45], [-0.45, -0.45], [-0.45, 0.45] ]" ).to_parameter_msg(),
            Parameter(name='width', value =  2).to_parameter_msg(),
            Parameter(name='height', value = 2).to_parameter_msg(),

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


def main(args=None):
    rclpy.init(args=args)
    # patrol_behavior_server  approach_shelf_server  shelf_position_server
    client_node = MinimalClientAsync()
    response = client_node.send_request()
    if response:
        client_node.get_logger().info('Service call succeeded')
    else:
        client_node.get_logger().info('Service call failed')

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
