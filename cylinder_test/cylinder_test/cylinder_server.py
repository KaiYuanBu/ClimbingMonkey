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

from cylinder_test.IDSServoDriver import IDSServoDriver
from dmke_interface.action import SetExtension

class CylinderServer(Node):
    """MonKey Arm cylinder control wrapper for IDSServoDriver."""

    def __init__(self):
        """MonKey Arm cylinder control node."""
        super().__init__('cylinder_action_server')

        self.declare_parameter('can_id', 1,
            ParameterDescriptor(description='CAN ID of the target driver.'))
        self.declare_parameter('can_channel', '/dev/ttyUSB2',
            ParameterDescriptor(description='Channel of CAN tranceiver.'))
        # self.declare_parameter('can_baudrate', 2000000,
        self.declare_parameter('can_baudrate', 115200,
            ParameterDescriptor(description='Baudrate of USB communication.'))
        self.declare_parameter('can_bitrate', 500000,
            ParameterDescriptor(description='Bitrate of CAN communication bus in bit/s.'))
        
        # Get parameter
        self.can_id = self.get_parameter('can_id').value
        can_channel = self.get_parameter('can_channel').value
        baudrate = self.get_parameter('can_baudrate').value
        bitrate = self.get_parameter('can_bitrate').value

        # Create CAN connection
        self.bus = can.ThreadSafeBus(
            interface='seeedstudio', channel=can_channel, baudrate=baudrate, bitrate=bitrate
            # interface='socketcan', channel='can0', bitrate=500000
        )

        time.sleep(0.5)

        # Initialize driver
        self.driver = IDSServoDriver(self.bus, self.can_id, name="LA1")
        self.driver.fault_reset()
        time.sleep(0.5)
        self.driver.set_positional_control_mode()

        # Declare action server
        self.action_server = ActionServer(self, SetExtension, 'SetExtension', self.action_callback)

        # Start message
        self.get_logger().info(f"SetExtension Server started")

    def action_callback(self, goal_handle:ServerGoalHandle, filepath='saved_extension.txt'):
        """Callback for set extension action."""
        self.get_logger().info("Executing goal...")

        # self.driver.set_positional_control_mode()
        
        self.driver.set_extension(goal_handle.request.target_extension, 8, False)
        # actual_ext = self.driver.extension_feedback()
        
        self.monitor_extension(goal_handle, self.driver, filepath, message=self.bus.recv())
        # prev_ext = float(self.driver.extension)

            # time.sleep(interval)
            
            
            # self.get_logger().info(f'Extension: {actual_ext}')
            # self.save_value_to_file(actual_ext, filepath)

            # if actual_ext < prev_ext:
            #     sub_pos = abs((prev_ext - actual_ext))
            #     file_pos =self.read_value_from_file(filepath) - sub_pos
            #     print(f"Actual extension: {file_pos}")
            #     feedback_msg.current_extension = file_pos
            #     self.save_value_to_file(file_pos, filepath)
            # #     feedback_msg.current_extension = file_pos
            # #     # save_value_to_file(file_pos, filepath)

            # elif actual_ext > prev_ext:
            #     sub_pos = abs((prev_ext - actual_ext))
            #     file_pos = self.read_value_from_file(filepath) + sub_pos
            #     print(f"Actual extension: {file_pos}")
            #     feedback_msg.current_extension = file_pos
            #     self.save_value_to_file(file_pos, filepath)
            # #     feedback_msg.current_extension = file_pos

            
            # # feedback_msg = SetExtension.Feedback()

            # # Check if position has changed significantly
            # if abs(actual_ext - prev_ext) < threshold:
            #     break

            # prev_ext = actual_ext
            # goal_handle.publish_feedback(feedback_msg)
            # goal_handle.publish_feedback(feedback_msg)
            # self.save_value_to_file(feedback_msg)        

        goal_handle.succeed()

        # self.driver.disable()

        result = SetExtension.Result()
        result.success = True
        return result
    
    def save_value_to_file(self, number, file_path):
        if not isinstance(number, float):
            raise ValueError("Input must be an value.")
            # print("Input not value...Saving as 0")

        with open(file_path, 'w') as file:
            file.write(str(number))

    def read_value_from_file(self, file_path):
        # default_value = 0
        # try:
        with open(file_path, 'r') as file:
            data = file.read().strip()
            return float(data)
        # except (ValueError, FileNotFoundError):
        #     # If file not found, create the file with default value
        #     with open(file_path, 'w') as file:
        #         file.write(str(default_value))
        #     return default_value

    def real_ext(self, instance, target_ext, file_path):
        starting_ext = self.read_value_from_file(file_path)
        read_ext = instance.read_actual_ext()


        if starting_ext - 0.1 <= read_ext <= starting_ext + 0.1:
            starting_ext = read_ext

        if not (starting_ext - 0.1 <= read_ext <= starting_ext + 0.1):
            starting_ext = starting_ext

        if read_ext is None:
            starting_ext = starting_ext

        print(f"Current extition: {starting_ext}")
        real_target_ext = read_ext + (target_ext - starting_ext)
        return real_target_ext


    # Threshold = 50 with Interval = 0.1 for 10000rps2 accel and decel
    def monitor_extension(self, goal_handle, instance, filepath, message, threshold=0.05, interval=0.1):
        """
        Monitors the extension of a servo motor continuously until the change
        in extension is less than the specified threshold.

        Args:
        - c1: The DMKEServoDriver2_V1 object for the servo motor.
        - threshold (optional): The threshold for considering the change in
                                extension significant. Defaults to 5.
        - interval (optional): The time interval (in seconds) between extension
                               readings. Defaults to 0.2 seconds.
        """

        prev_ext = instance.extension_feedback(message)
        # print(f"extension from Motor's Perspective {prev_ext}")
        error_count = 0  # Initialize error count
        feedback_msg = SetExtension.Feedback()

        while self.driver.is_running:
            time.sleep(interval)
            try:
                # Read the actual extension
                actual_ext = instance.extension_feedback(message)

                # Handle the case where actual_ext is a valid extension
                if actual_ext is not None and prev_ext is not None:

                    if actual_ext < prev_ext:
                        sub_ext = abs((prev_ext - actual_ext))
                        file_ext =self.read_value_from_file(filepath) - sub_ext
                        print(f"Actual extension: {file_ext}")
                        # save_value_to_file(file_ext, filepath)
                        feedback_msg.current_extension = file_ext
                        self.save_value_to_file(file_ext, filepath)

                    elif actual_ext > prev_ext:
                        sub_ext = abs((prev_ext - actual_ext))
                        file_ext = self.read_value_from_file(filepath) + sub_ext
                        print(f"Actual extension: {file_ext}")
                        feedback_msg.current_extension = file_ext
                        self.save_value_to_file(file_ext, filepath)

                    goal_handle.publish_feedback(feedback_msg)

                    # Check if extension has changed significantly
                    if abs(actual_ext - prev_ext) < threshold:
                        # read_ext = instance.read_actual_ext()
                        x = abs(actual_ext - prev_ext)
                        if actual_ext < prev_ext:
                            file_ext = self.read_value_from_file(filepath) - x
                            self.save_value_to_file(file_ext, filepath)
                        elif actual_ext > prev_ext:
                            file_ext = self.read_value_from_file(filepath) + x
                            self.save_value_to_file(file_ext, filepath)
                        
                        print(actual_ext)
                        break

                    prev_ext = actual_ext

                else:
                    # raise ValueError("Failed to read current extension of motor")
                    continue

            except can.CanError as e:
                error_code = e.code
                print("CAN Error Code:", hex(error_code))
                error_count += 1  # Increase error count on each occurrence

                if error_count >= 5:
                    raise ValueError("CAN Error occurred 5 times. Stopping the function.")

                else:
                    # self.monitor_extension(self, goal_handle, instance, filepath, threshold=3, interval=0.05)
                    continue

    # SERVICE #
class CheckExtensionService(Node):

    def __init__(self):
        super().__init__('check_extension_service')
        self.srv = self.create_service(GetExtension, 'GetExtension', self.get_extension_callback)        # CHANGE
        self.get_logger().info('GetExtension Server is ready!')


    def get_extension_callback(self, response):
        # node_id = request.node_id.to_bytes(1, byteorder='little')
        response.extension = read_value_from_file('saved_extension_pos.txt')                                           # CHANGE
        # self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response
    


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
