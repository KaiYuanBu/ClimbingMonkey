"""
ROS2 Interface for Monkey Arm cylinder control.

Written by:
    1. Kean Hao
Last Updated: 8 September 2023

Usage (send goal):
ros2 action send_goal -f /arm/bearing/SetExtension arm_interfaces/action/SetExtension "{target_rotation: 10.0}"
ros2 action send_goal -f /arm/cylinder_1/SetExtension arm_interfaces/action/SetExtension "{target_extension: 0.3}"
os2 action send_goal -f /arm/cylinder_2/SetExtension arm_interfaces/action/SetExtension "{target_extension: 0.3}"
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

        self.declare_parameter('can_id', 71,
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

    def action_callback(self, goal_handle:ServerGoalHandle):
        """Callback for set extension action."""
        self.get_logger().info("Executing goal...")

        self.driver.set_extension(goal_handle.request.target_extension, 8, False)

        feedback_msg = SetExtension.Feedback()
        while self.driver.is_running:
            self.get_logger().info(f'Extension: {self.driver.extension}')
            feedback_msg.current_extension = float(self.driver.extension)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()

        result = SetExtension.Result()
        result.success = True
        return result

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
