from dmke_interface.srv import GetPosition      # CHANGE
import sys
import rclpy
from rclpy.node import Node


class GetPosClientAsync(Node):

    def __init__(self):
        super().__init__('get_pos_client')
        self.cli = self.create_client(GetPosition, 'get_position_client')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')                              # CHANGE

    # def send_request(self):
    #     self.req.a = int(sys.argv[1])
    #     self.req.b = int(sys.argv[2])
    #     self.req.c = int(sys.argv[3])                  # CHANGE
    #     self.future = self.cli.call_async(self.req)
            
    def getpos_service_call(self):
        request = GetPosition.Request()
        future = self.cli.call_async(request)
        self.get_logger().info('Calling Trigger Service')
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().result
        else:
            self.get_logger().error('Service call failed')
            return 0  # Default integer value if service call failed


def main(args=None):
    rclpy.init(args=args)

    getpos_client = GetPosClientAsync()
    # minimal_client.send_request()

    # while rclpy.ok():
    # rclpy.spin_once(getpos_client)
    try:
        # response = getpos_client.future.result()
        response = getpos_client.getpos_service_call()
    except Exception as e:
        getpos_client.get_logger().info(
            'Service call failed %r' % (e,))
    else:
        getpos_client.get_logger().info(
            'Sending Position Obtained to Client: %u ' %                               # CHANGE
            (response)) # CHANGE

    getpos_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
