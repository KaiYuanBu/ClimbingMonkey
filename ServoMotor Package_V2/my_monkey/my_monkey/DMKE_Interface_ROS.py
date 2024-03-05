from rclpy.action import ActionServer
from rclpy.node import Node
# from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import ParameterDescriptor


import time
import canopen
import rclpy
from rclpy.action.server import ServerGoalHandle

from DMKEServoDriver2 import DMKEServoDriver2_V1
from DMKE_action.action import SetPosition


class DMKEInterface(Node):
    def __init__(self):
        super().__init__('DMKE_servo')

        # Initialize driver
        # self.get_logger().info(f"Setting up DMKE servo driver with NODE ID {self.node_id}")
        self.network = canopen.Network()
        self.network.connect(interface='seeedstudio', channel='COM7', baudrate=115200, bitrate=500000)
        time.sleep(1)

        self.c1 = DMKEServoDriver2_V1(self.network, 0x01)
        
        qos_profile = QoSProfile(depth=10)
        self._action_server = ActionServer(
            self,
            SetPosition,
            'SetPosition',
            self.execute_callback,
            qos_profile
        )
        
        # self.declare_parameter('node_id', 0x01,
        #     ParameterDescriptor(description='NODE ID of the target driver.'))
        # self.declare_parameter('can_channel', 'COM7', # can*, vcan* or /dev/ttyUSB*
        #     ParameterDescriptor(description='Channel of CAN tranceiver.'))
        # self.declare_parameter('can_baudrate', 115200,
        #     ParameterDescriptor(description='Baudrate of USB communication.'))
        # self.declare_parameter('can_bitrate', 500000,
        #     ParameterDescriptor(description='Bitrate of CAN communication bus in bit/s.'))
        
        # # Get parameters
        # self.node_id = self.get_parameter('node_id').value
        # can_channel = self.get_parameter('can_channel').value
        # baudrate = self.get_parameter('can_baudrate').value
        # bitrate = self.get_parameter('can_bitrate').value

         # Declare action server
        # self.action_server = ActionServer(self, SetPosition, f'DMKE_servo/SetPosition', self.target_action_callback)
    

        time.sleep(0.5)

        
        
    # def execute_callback(self, goal_handle):
    #     self.get_logger().info('Executing goal...')
    #     result = SetPosition.Result()
    #     return result
        
    def target_action_callback(self, goal_handle:ServerGoalHandle):
        """Callback for set position action."""

        # Target Positions
        # self.c1.set_target_position(goal_handle.request.targetposition1)
        # self.c1.set_target_position2(goal_handle.request.targetposition2)
        # self.c1.set_target_position3(goal_handle.request.targetposition3)
        # self.c1.set_target_position4(goal_handle.request.targetposition4)

        # FEEDBACKS
        feedback_msg = SetPosition.Feedback()
        # feedback_msg2 = SetPosition.Feedback2()
        # feedback_msg3 = SetPosition.Feedback3()
        # feedback_msg4 = SetPosition.Feedback4()

        # RESULTS
        result = SetPosition.Result()
        # result2 = SetPosition.Result2()
        # result3 = SetPosition.Result3()
        # result4 = SetPosition.Result4()

        try:


            self.get_logger().info("Executing target...")

            # CLEAR FAULTS
            self.c1.NMT_Reset_Node()
            # c1.NMT_Reset_Comm()

            # PRE-OPERATIONAL MODE
            self.c1.NMT_Pre_Op()
            time.sleep(2)

            # START OPERATION
            self.c1.NMT_Start()
            time.sleep(2)

            # ENABLING DRIVER
            self.c1.enable()

            # READING CURRENT POSITION
            actual_pos = self.c1.read_actual_pos()
            print("Current Position = ", actual_pos)

            # SETTING POSITION CONTROL MODE
            print("Setting positional control mode")
            self.c1.set_pos_control_mode()
            time.sleep(2)

            # SETTING PARAMETERS OF POSITION CONTROL MODE
            print("Setting Parameters for position control mode")
            self.c1.set_profile_velocity(2500)
            self.c1.set_profile_acceleration(1000)
            self.c1.set_profile_deceleration(1000)
            time.sleep(2)


            # == POSITION 1 == #
            # Move to target position 1
            print(f"Setting Target position to {goal_handle.request.targetposition1}")
            self.c1.set_target_location(goal_handle.request.targetposition1)
            time.sleep(2)

            # Absolute Position Start Trigger
            print(f"Moving to position {goal_handle.request.targetposition1}")
            self.c1.start_trigger_absolute()

            # Provide feedback that the position has been set
            # feedback_msg1.feedback1 = True

            self.monitor_position(goal_handle, self.c1, feedback_msg1, result1)


            # Continuous Positon Feedback
            # self.monitor_position(self.c1, threshold=5, interval=0.2)
            # Get and send the current motor position as feedback
            # current_position = self.c1.read_actual_pos()
            # feedback_msg1.current_position = current_position
            # goal_handle.publish_feedback(feedback_msg)

            # Set the result for target_position1 as success
            # result1.success1 = True


            # DISABLING DRIVER
            time.sleep(2)
            self.c1.disable()
            time.sleep(2)


            # == POSITION 2 == #
            # RE-ENABLE DRIVER
            self.c1.enable()

            # Move to target position 2
            print(f"Setting Target position to {goal_handle.request.targetposition2}")
            self.c1.set_target_location(goal_handle.request.targetposition2)
            time.sleep(2)

            # Absolute Position Start Trigger
            print(f"Moving to position {goal_handle.request.targetposition2}")
            self.c1.start_trigger_absolute()

            # Continuous Positon Feedback

            self.monitor_position(goal_handle, self.c1, feedback_msg2, result2)

            # DISABLING DRIVER
            time.sleep(2)
            self.c1.disable()
            time.sleep(2)


            # == POSITION 3 == #
            # RE-ENABLE DRIVER
            self.c1.enable()

            # Move to target position 3
            print(f"Setting Target position to {goal_handle.request.targetposition3}")
            self.c1.set_target_location(goal_handle.request.targetposition3)
            time.sleep(2)

            # Absolute Position Start Trigger
            print(f"Moving to position {goal_handle.request.targetposition3}")
            self.c1.start_trigger_absolute()

            # Continuous Positon Feedback
            self.monitor_position(goal_handle, self.c1, feedback_msg3, result3)

            # DISABLING DRIVER
            time.sleep(2)
            self.c1.disable()
            time.sleep(2)


            # == POSITION 3 == #
            # RE-ENABLE DRIVER
            self.c1.enable()

            # Move to target position 4
            print(f"Setting Target position to {goal_handle.request.targetposition4}")
            self.c1.set_target_location(goal_handle.request.targetposition4)
            time.sleep(2)

            # Absolute Position Start Trigger
            print(f"Moving to position {goal_handle.request.targetposition4}")
            self.c1.start_trigger_absolute()

            # Continuous Positon Feedback
            self.monitor_position(goal_handle, self.c1, feedback_msg4, result4)

            # DISABLING DRIVER
            time.sleep(2)
            self.c1.disable()
            time.sleep(2)


            print("Returning to PreOperational State")
            self.c1.NMT_Pre_Op()
            time.sleep(2)

            print("Entering Stop State")
            self.c1.NMT_Stop()
            time.sleep(2)


            # feedback_msg = SetPosition.Feedback()
            # result = SetPosition.Result()
            # result.success = True

        except canopen.sdo.SdoCommunicationError:
            print("Failed to update node timeout protection")


    def monitor_position(self, goal_handle, instance, feedback_msg, result):

        prev_time = time.time()

        # Loop until target reached or timeout
        while instance.is_running:
            # Get positional feedback
            self.get_logger().info(f'Position: {instance.current_position}')
            feedback_msg.current_position = float(instance.current_position)
            goal_handle.publish_feedback(feedback_msg)

            # Check for timeout
            if (time.time() - prev_time) >= 8:
                result.success = False
                goal_handle.abort()
                instance.is_running = False
                # TODO: servo motor rotation halted
                break

            time.sleep(0.1)

        if result.success:
            goal_handle.succeed()
        
        return result
    
    # def monitor_position(instance, threshold=5, interval=0.2):
    #     """
    #     Monitors the position of a servo motor continuously until the change
    #     in position is less than the specified threshold.

    #     Args:
    #     - c1: The DMKEServoDriver2_V1 object for the servo motor.
    #     - threshold (optional): The threshold for considering the change in
    #                             position significant. Defaults to 5.
    #     - interval (optional): The time interval (in seconds) between position
    #                            readings. Defaults to 0.2 seconds.
    #     """
    #     prev_pos = instance.read_actual_pos()
    #     while True:
    #         time.sleep(interval)
    #         actual_pos = instance.read_actual_pos()

    #         if actual_pos is not None and prev_pos is not None:
    #             print(f"Actual position: {actual_pos}")

    #             # Check if position has changed significantly
    #             if abs(actual_pos - prev_pos) < threshold:
    #                 break
                
    #         prev_pos = actual_pos

    def destroy(self):
        # Clean up CANopen network
        self.c1.disable()
        self.network.disconnect()
        super().destroy_node()


def main(args=None):
    """Run when this script is called."""
    rclpy.init(args=args)

    node = DMKEInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy()


    print("Task completed successfully")
    # network.disconnect()


if __name__ == '__main__':
    main()