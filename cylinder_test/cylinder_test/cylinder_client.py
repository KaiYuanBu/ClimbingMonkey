import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dmke_interface.action import SetExtension

class CylinderClient(Node):

    def __init__(self):
        super().__init__('cylinder_action_client')
        self._action_client = ActionClient(self, SetExtension, 'SetExtension')

    def send_goal(self, target_extension):
        goal_msg = SetExtension.Goal()
        goal_msg.target_extension = target_extension

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
            self.get_logger().info('Result received: {0}'.format(result.success_ext))
        except Exception as e:
            self.get_logger().info('Exception while calling service: %r' % (e,))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: Current extension is {0}'.format(feedback.current_extension))

def main(args=None):
    rclpy.init(args=args)

    action_client = CylinderClient()

    target_extension = 0.0 # Example target extension in meters
    action_client.send_goal(target_extension)

    rclpy.spin(action_client)


# Based on black line
    # 0.0 = 0.0
    # 0.1 = 7.5cm
    # 0.2 = 14.6cm
    # 0.3 = 21.9cm
    # 0.4 = 29cm
    # 0.5 = 36.1cm
    # 0.6 = 43.2cm
    # 0.7 = 50.5cm
    # 0.8 = 57.6cm
    # 0.9 = 64.7cm
    # 1.0 = 71.8cm
    # 1.1 = 79.1cm
    # 1.2 = 86.2cm
    # 1.3 = 93.4cm
    # 1.4 = 95.7cm (1m)
    # Distance Between Red and Black Line = 3.7cm

if __name__ == '__main__':
    main()