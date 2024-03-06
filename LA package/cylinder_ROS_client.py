import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dmke_interface.action import SetExtension

class CylinderClient(Node):

    def __init__(self):
        super().__init__('cylinder_action_client')
        self._action_client = ActionClient(self, SetExtension, f'{self.get_name()}/SetExtension', self.action_callback)

    def send_goal(self, target_extension):
        goal_msg = SetExtension.Goal()
        goal_msg.target = target_extension

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info('Result received: {0}'.format(result.success))
        except Exception as e:
            self.get_logger().info('Exception while calling service: %r' % (e,))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: Current extension is {0}'.format(feedback.current_extension))

def main(args=None):
    rclpy.init(args=args)

    action_client = CylinderClient()

    target_extension = 0.5 # Example target extension in meters
    action_client.send_goal(target_extension)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()