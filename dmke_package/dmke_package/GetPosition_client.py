import sys

from dmke_interface.srv import GetPosition
import rclpy
from rclpy.node import Node


class GetPositionClient(Node):

    def __init__(self):
        super().__init__('get_position_client')
        self.cli = self.create_client(GetPosition, 'get_position')
        while not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPosition.Request()

    def call_service(self):
        self.get_logger().info('Calling the empty service...')
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.position:
                self.get_logger().info(f'Position Obtained: {response}')
            else:
                self.get_logger().info('Service call failed!')
        else:
            self.get_logger().info('Service call failed: Timeout exceeded')



def main(args=None):
    rclpy.init(args=args)
    getpos_client = GetPositionClient()
    getpos_client.call_service()
    rclpy.spin(getpos_client)
    # getpos_client.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()