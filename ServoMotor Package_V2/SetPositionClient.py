import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dmke_interface.action import SetPosition

class SetPositionActionClient(Node):

    def __init__(self):
        super().__init__('set_position_action_client')
        self._action_client = ActionClient(self, SetPosition, 'set_position')

    def send_goal(self, target_position):
        goal_msg = SetPosition.Goal()
        goal_msg.target_position = target_position

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, 
                                                                     feedback_callback=self.feedback_callback)

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
        self.get_logger().info('Received feedback: Current position is {0}'.format(feedback.current_position))

def main(args=None):
    rclpy.init(args=args)

    action_client = SetPositionActionClient()

    target_position = 500000 # Example target position
    action_client.send_goal(target_position)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()