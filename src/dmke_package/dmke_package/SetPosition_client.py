import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_interface.action import SetPosition

# /home/bky/my_monkey/src/dmke_package/action

class SetPositionClient(Node):

    def __init__(self):
        super().__init__('set_position_client')
        self._action_client = ActionClient(self, SetPosition, 'set_position')
        self.get_logger().info('Action Client is up!')

    def send_goal(self, target):
        # List of target positions to send sequentially
        # target_positions = [200000, 400000, -200000, 0]
        # self.target_position = 200000
        self.request_msg = SetPosition.Goal()
        self.request_msg.targetposition = target

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(self.request_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # if future.result() is not None:
        #     result = future.result().result.resultstuff
        #     self.get_logger().info(f'Result of action: {result}')
        # else:
        #     self.get_logger().error('Failed to get result')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

        # if not (self.request_msg.targetposition - 5 < self.current_pos < self.request_msg.targetposition + 5): 
        #     request_msg.targetposition = 

        # else:
        #     print("Task Completed")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pos = feedback.currentposition
        self.get_logger().info(f'Received feedback position: {current_pos}')

        # if self.request_msg.targetposition - 5 < self.current_pos < self.request_msg.targetposition + 5:  # Example condition, modify as needed
        #     self.get_logger().info("Current position is close to target, canceling goal...")
        #     self._action_client.cancel_goal_async()

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")
        rclpy.shutdown()
        # Callback function to handle result
    #     def goal_response_callback(future):
    #         goal_handle = future.result()
    #         if not goal_handle.accepted:
    #             self.get_logger().info('Goal rejected :(')
    #             return

    #         self.get_logger().info('Goal accepted :)')

    #         # Move to the next goal
    #         self.current_goal_index += 1

    #         # If there are more positions, send the next goal
    #         if self.current_goal_index < len(target_positions):
    #             next_goal = SetPosition.Goal()
    #             next_goal.targetposition = target_positions[self.current_goal_index]
    #             self._send_goal(next_goal, goal_response_callback)
    #         else:
    #             self.get_logger().info('All goals achieved!')


    #     def result_callback(self, future):
    #         result = future.result().result
    #         self.get_logger().info('Result: {0}'.format(result))
    #         # rclpy.shutdown()

    #     def feedback_callback(self, feedback_msg):
    #         feedback = feedback_msg.feedback
    #         self.get_logger().info('Received feedback: {0}'.format(feedback))


    #     # Create the first goal message
    #     first_goal = SetPosition.Goal()
    #     first_goal.targetposition = target_positions[self.current_goal_index]

       
    #     # Send the first goal asynchronously
    #     self._action_client.wait_for_server()
    #     self._send_goal(first_goal, goal_response_callback, feedback_callback, result_callback)


    # def _send_goal(self, goal_msg, callback):
    #     self.get_logger().info('Sending goal request...')
    #     self._action_client.wait_for_server()
    #     self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback, done_callback=callback)

    

# def main(args=None):
#     rclpy.init(args=args)
#     client = SetPositionClient()

#     try:
#         client.send_goal()
#     except KeyboardInterrupt:
#         pass

#     client.destroy_node()
#     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = SetPositionClient()  # Create an instance of your action client class

    # try:
    action_client.send_goal(500000)  # Call the send_goal function to start the action

    rclpy.spin(action_client)  # Spin the node to keep it running until interrupted

    # except KeyboardInterrupt:
    #     pass

    # action_client.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
