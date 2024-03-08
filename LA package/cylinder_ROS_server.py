"""
ROS2 Interface for Monkey Arm cylinder control.

Written by:
    1. Kean Hao
Last Updated: 8 September 2023

Usage (send goal):
ros2 action send_goal -f /arm/bearing/SetExtension arm_interfaces/action/SetExtension "{target_rotation: 10.0}"
ros2 action send_goal -f /arm/cylinder_1/SetExtension arm_interfaces/action/SetExtension "{target_extension: 0.3}"
ros2 action send_goal -f /arm/cylinder_2/SetExtension arm_interfaces/action/SetExtension "{target_extension: 0.3}"
"""

from rclpy.action import ActionServer
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor #, SetParametersResult

import time
import can
import rclpy
from rclpy.action.server import ServerGoalHandle

from IDSServoDriver import IDSServoDriver
from arm_interfaces.action import SetExtension

class CylinderServer(Node):
    """MonKey Arm cylinder control wrapper for IDSServoDriver."""

    def __init__(self):
        """MonKey Arm cylinder control node."""
        super().__init__('cylinder_action_server')

        self.declare_parameter('can_id', 1,
            ParameterDescriptor(description='CAN ID of the target driver.'))
        self.declare_parameter('can_channel', '/dev/ttyS0',
            ParameterDescriptor(description='Channel of CAN tranceiver.'))
        self.declare_parameter('can_baudrate', 2000000,
            ParameterDescriptor(description='Baudrate of USB communication.'))
        self.declare_parameter('can_bitrate', 500000,
            ParameterDescriptor(description='Bitrate of CAN communication bus in bit/s.'))
        
        # Get parameters
        self.can_id = self.get_parameter('can_id').value
        can_channel = self.get_parameter('can_channel').value
        baudrate = self.get_parameter('can_baudrate').value
        bitrate = self.get_parameter('can_bitrate').value

        # Create CAN connection
        bus = can.ThreadSafeBus(
            interface='seeedstudio', channel=can_channel, baudrate=baudrate, bitrate=bitrate
            # interface='socketcan', channel='can0', bitrate=500000
        )

        time.sleep(0.5)

        # Initialize driver
        self.driver = IDSServoDriver(bus, self.can_id, name=self.get_name())
        time.sleep(0.5)
        self.driver.set_positional_control_mode()

        # Declare action server
        self.action_server = ActionServer(self, SetExtension, f'{self.get_name()}/SetExtension', self.action_callback)

        # Start message
        self.get_logger().info(f"{self.get_name()} started")

    def action_callback(self, goal_handle:ServerGoalHandle, filepath='saved_extension.txt', interval=0.1, threshold=0.05):
        """Callback for set extension action."""
        self.get_logger().info("Executing goal...")

        self.driver.fault_reset()

        self.driver.set_positional_control_mode()
        
        self.driver.set_extension(goal_handle.request.target_extension, 8, False)

        feedback_msg = SetExtension.Feedback()
        prev_ext = float(self.driver.extension)

        while self.driver.is_running:
            time.sleep(interval)
            
            actual_ext = float(self.driver.extension)
            self.get_logger().info(f'Extension: {actual_ext}')
            feedback_msg.current_extension = actual_ext

            if actual_ext < prev_ext:
                sub_pos = abs((prev_ext - actual_ext))
                file_pos =self.read_integer_from_file(filepath) - sub_pos
                print(f"Actual extension: {file_pos}")
                # save_integer_to_file(file_pos, filepath)

            elif actual_ext > prev_ext:
                sub_pos = abs((prev_ext - actual_ext))
                file_pos = self.read_integer_from_file(filepath) + sub_pos
                print(f"Actual extension: {file_pos}")

            self.save_integer_to_file(file_pos, filepath)
            feedback_msg = SetExtension.Feedback()
            feedback_msg.current_extension = file_pos
            goal_handle.publish_feedback(feedback_msg)

            # Check if position has changed significantly
            if abs(actual_ext - prev_ext) < threshold:
                break

            prev_ext = actual_ext

            # goal_handle.publish_feedback(feedback_msg)
            self.save_value_to_file(feedback_msg)        

        goal_handle.succeed()

        result = SetExtension.Result()
        result.success = True
        return result
    
    def save_value_to_file(self, number, file_path):
        if not isinstance(number, float):
            raise ValueError("Input must be a float.")
            # print("Input not integer...Saving as 0")

        with open(file_path, 'w') as file:
            file.write(str(number))

    def read_value_from_file(self, file_path):
        # default_value = 0
        try:
            with open(file_path, 'r') as file:
                data = file.read().strip()
                return float(data)
        
        except FileNotFoundError:
            # If the file does not exist, create it and return 0
            self.get_logger().warn(f"File {file_path} not found. Creating it with default value 0.")
            self.save_integer_to_file(0, file_path)
            return 0
    
    # def monitor_position(self, goal_handle, instance, filepath, threshold=3, interval=0.1):
    #     """
    #     Monitors the position of a servo motor continuously until the change
    #     in position is less than the specified threshold.

    #     Args:
    #     - instance: The instance for the linear actuator.
    #     - threshold (optional): The threshold for considering the change in
    #                             position significant. Defaults to 5.
    #     - interval (optional): The time interval (in seconds) between position
    #                            readings. Defaults to 0.2 seconds.
    #     """

    #     prev_pos = instance.read_actual_pos()
    #     # print(f"Position from Motor's Perspective {prev_pos}")
    #     error_count = 0  # Initialize error count

    #     while True:
    #         time.sleep(interval)
    #         try:
    #             # Read the actual position
    #             actual_pos = instance.read_actual_pos()

    #             # Handle the case where actual_pos is a valid position
    #             if actual_pos is not None and prev_pos is not None:

    #                 if actual_pos < prev_pos:
    #                     sub_pos = abs((prev_pos - actual_pos))
    #                     file_pos =self.read_integer_from_file(filepath) - sub_pos
    #                     print(f"Actual extension: {file_pos}")
    #                     # save_integer_to_file(file_pos, filepath)

    #                 elif actual_pos > prev_pos:
    #                     sub_pos = abs((prev_pos - actual_pos))
    #                     file_pos = self.read_integer_from_file(filepath) + sub_pos
    #                     print(f"Actual extension: {file_pos}")

    #                 self.save_integer_to_file(file_pos, filepath)
    #                 feedback_msg = SetExtension.Feedback()
    #                 feedback_msg.current_position = file_pos
    #                 goal_handle.publish_feedback(feedback_msg)

    #                 # Check if position has changed significantly
    #                 if abs(actual_pos - prev_pos) < threshold:
    #                     read_pos = instance.read_actual_pos()
    #                     print(read_pos)
    #                     break

    #                 prev_pos = actual_pos

    #             else:
    #                 # raise ValueError("Failed to read current position of motor")
    #                 continue

    #         except can.CanTimeoutError as e:
    #             error_code = e.code
    #             print("SDO Aborted Error Code:", hex(error_code))
    #             error_count += 1  # Increase error count on each occurrence

    #             if error_count >= 5:
    #                 raise ValueError("SDO Aborted Error occurred 5 times. Stopping the function.")

    #             else:
    #                 continue

def main(args=None):
    """Run when this script is called."""
    rclpy.init(args=args)

    #Setup CAN listener
    print("Setting CAN notifier")
    cylinder = CylinderServer()

    try:
        rclpy.spin(cylinder)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cylinder.destroy_node()

if __name__ == '__main__':
    main()
