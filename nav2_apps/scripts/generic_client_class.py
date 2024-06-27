import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class GenericServiceClient(Node):

    def __init__(self, node_name, server_name):
        super().__init__(node_name)
        self.server_name = server_name
        self.cli = self.create_client(Empty, server_name)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Empty.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    # patrol_behavior_server  approach_shelf_server  shelf_position_server
    client_node = GenericServiceClient('generic_service_client_node', 'shelf_position_server')
    response = client_node.send_request()
    if response:
        client_node.get_logger().info('Service call succeeded')
    else:
        client_node.get_logger().info('Service call failed')

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
