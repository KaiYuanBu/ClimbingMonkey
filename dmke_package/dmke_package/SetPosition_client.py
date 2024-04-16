# import rclpy
# import time
# from rclpy.action import ActionClient
# from dmke_interface.action import SetPosition

# # def main(args=None):
# #     rclpy.init(args=args)

# #     action_client = ActionClient(rclpy.create_node('set_position_action_client'), SetPosition, 'set_position')

# #     goal_msg = SetPosition.Goal()
# #     goal_msg.target_position = 100  # Set your desired target position here

# #     goal_handle_future = action_client.send_goal_async(goal_msg)

# #     # Wait for the action server to be available
# #     while not goal_handle_future.done():
# #         rclpy.spin_once(action_client._node)
# #         time.sleep(0.1)

# #     try:
# #         goal_handle = goal_handle_future.result()
# #     except Exception as e:
# #         print(f"Failed to get result: {e}")
# #     else:
# #         if goal_handle.accepted:
# #             print("Goal accepted!")
# #         else:
# #             print("Goal rejected!")

# #         # Wait for the result
# #         while goal_handle.status != goal_handle.STATUS_SUCCEEDED and goal_handle.status != goal_handle.STATUS_ABORTED:
# #             rclpy.spin_once(action_client._node)
# #             time.sleep(0.1)

# #         if goal_handle.status == goal_handle.STATUS_SUCCEEDED:
# #             print("Goal succeeded!")
# def main(args=None):
#     rclpy.init(args=args)
#     print("Initialized ROS 2")

#     action_client = ActionClient(rclpy.create_node('set_position_action_client'), SetPosition, 'set_position')
#     print("Created Action Client")

#     goal_msg = SetPosition.Goal()
#     goal_msg.target_position = 100  # Set your desired target position here
#     print("Created Goal Message")

#     goal_handle_future = action_client.send_goal_async(goal_msg)
#     print("Sent Goal")

#     try:
#         rclpy.spin_until_future_complete(action_client._node, goal_handle_future)
#         goal_handle = goal_handle_future.result()
#         print("Goal Complete")
#     except Exception as e:
#         print(f"Failed to get result: {e}")

#     if goal_handle is not None:
#         if goal_handle.accepted:
#             print("Goal accepted!")
#         else:
#             print("Goal rejected!")

#         if goal_handle == True:
#             print("Goal succeeded!")
#             print("Current Position:", goal_handle.feedback.current_position)
#         else:
#             print("Goal aborted or failed!")
#     else:
#         print("Goal handle is None. Exiting...")

#     action_client.destroy()
#     rclpy.shutdown()
#     print("Shutting down")

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dmke_interface.action import SetPosition
from dmke_interface.srv import GetPosition
# import rclpy
# from rclpy.node import Node

class SetPositionActionClient(Node):

    def __init__(self):
        super().__init__('set_position_action_client')
        self._action_client = ActionClient(self, SetPosition, 'set_position')

    def send_goal(self, target_position):
        goal_msg = SetPosition.Goal()
        goal_msg.target_position = target_position

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
        self.get_logger().info('Received feedback: Current position is {0}'.format(feedback.current_position))



class GetPositionClient(Node):

    def __init__(self):
        super().__init__('get_position_client')
        self.cli = self.create_client(GetPosition, 'get_position_client')
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

    action_client = SetPositionActionClient()

    #Close firmly on palm stem  (bottom only) = 170000 (~~50A should jump out)
    target_position = 100000 # Example target position
    action_client.send_goal(target_position)
    getpos_client = GetPositionClient()
    
    rclpy.spin(getpos_client)
    rclpy.spin(action_client)

    # getpos_client.destroy_node()

if __name__ == '__main__':
    main()