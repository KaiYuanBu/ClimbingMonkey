import rclpy
import canopen
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from action_interface.action import SetPosition
from .DMKEServoDriver2 import DMKEServoDriver2_V1

class SetPositionServer(Node):
    def __init__(self):

        self.network = canopen.Network()
        self.network.connect(interface='seeedstudio', channel='/dev/ttyS0', baudrate=115200, bitrate=500000)
        time.sleep(1)

        self.c1 = DMKEServoDriver2_V1(self.network, 0x01)

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

        self._action_server = ActionServer(
            self,
            SetPosition,
            'set_position',
            self.execute_callback)
        self.get_logger().info("Action Server is up!")


    def execute_callback(self, goal_handle):
        # This function is called when a new goal is received

        try:
            # Access the goal request
            # goal = goal_handle.request
            self.get_logger().info('Executing goal...')

            # == POSITION 1 == #
            # Move to target position 1
            targetposition1 = goal_handle.request.targetposition
            print(f"Setting Target position to {targetposition1}")
            self.c1.set_target_location(targetposition1)
            time.sleep(2)

            # Absolute Position Start Trigger
            print(f"Moving to position {targetposition1}")
            self.c1.start_trigger_absolute()

            # Provide feedback that the position has been set
            # feedback_msg1.feedback1 = True

            self.monitor_position(goal_handle, self.c1, threshold=5, interval=0.2)

            # DISABLING DRIVER
            time.sleep(2)
            self.c1.disable()
            time.sleep(2)


            self.c1.enable()

            self.c1.set_target_location(0)
            time.sleep(2)

            # Absolute Position Start Trigger
            print(f"Moving to position 0")
            self.c1.start_trigger_absolute()

            self.monitor_position(goal_handle, self.c1, threshold=5, interval=0.2)

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

            goal_handle.succeed()

            # feedback_msg = SetPosition.Feedback()
            result = SetPosition.Result()
            result.resultstuff = "Task Completed Successfully"
            return result

            # result = Fibonacci.Result()
            # result.sequence = feedback_msg.partial_sequence
            # return result

            
        except canopen.sdo.SdoCommunicationError:
            print("Failed to update node timeout protection")


    def monitor_position(self, goal_handle, instance, threshold=5, interval=0.2):

        prev_time = time.time()

        # Loop until target reached or timeout
        prev_pos = instance.read_actual_pos()
        while True:
            time.sleep(interval)
            actual_pos = instance.read_actual_pos()

            if actual_pos is not None and prev_pos is not None:
                print(f"Actual position: {actual_pos}")

                # Check if position has changed significantly
                if abs(actual_pos - prev_pos) < threshold:
                    break
                
            prev_pos = actual_pos

            # Update the feedback message
            feedback_msg = SetPosition.Feedback()
            feedback_msg.currentposition = actual_pos
            goal_handle.publish_feedback(feedback_msg)


    # def destroy(self):
    #     # Clean up CANopen network
    #     # self.c1.disable()
    #     self.network.disconnect()
    #     super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    set_position_server = SetPositionServer()
    rclpy.spin(set_position_server)

if __name__ == '__main__':
    main()
