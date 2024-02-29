from rclpy.action import ActionServer
from rclpy.node import Node
# from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import ParameterDescriptor
# from std_msgs.msg import String
from std_msgs.msg import Int32

import time
import canopen
import rclpy
from rclpy.action.server import ServerGoalHandle

from DMKEServoDriver.DMKEServoDriver import DMKEServoDriver2_V1
# from DMKE_action.action import SetPosition


class ClampNode(Node):
    def __init__(self):
        super().__init__('DMKE_servo_publisher')

        # Initialize driver
        # self.get_logger().info(f"Setting up DMKE servo driver with NODE ID {self.node_id}")
        self.network = canopen.Network()
        self.network.connect(interface='seeedstudio', channel='/dev/ttyS0', baudrate=115200, bitrate=500000)
        time.sleep(1)

        self.c1 = DMKEServoDriver2_V1(self.network, 0x01)
        
        
        self.publisher_ = self.create_publisher(Int32, '/motor/target_position', QoSProfile(depth=10))
        # self.publish_position(self.c1)
        # self.timer = self.create_timer(10, lambda:self.publish_position(self.c1))
        
        self.target_positions = [100000, 300000, 500000, 0]  # List of target positions
        # self.i = 0
        # self.is_movement_finished = True

        # CLEAR FAULTS
        self.c1.NMT_Reset_Node()
        self.c1.NMT_Reset_Comm()

        # PRE-OPERATIONAL MODE
        self.c1.NMT_Pre_Op()
        # time.sleep(2)

        # START OPERATION
        self.c1.NMT_Start()
        # time.sleep(2)

        # ENABLING DRIVER
        self.c1.enable()

        # SETTING POSITION CONTROL MODE
        print("Setting positional control mode")
        self.c1.set_pos_control_mode()
        time.sleep(0.5)

        # SETTING PARAMETERS OF POSITION CONTROL MODE
        print("Setting Parameters for position control mode")
        self.c1.set_profile_velocity(2500)
        # time.sleep(0.5)
        self.c1.set_profile_acceleration(1000)
        # time.sleep(0.5)
        self.c1.set_profile_deceleration(1000)
        # time.sleep(2)

        # Subscribe to the target position topic
        self.subscription = self.create_subscription(
            Int32,
            '/motor/target_position',
            self.target_callback,
            10  # QoS Profile, adjust as needed
        )

        self.current_target_index = 0
        self.is_movement_finished = True
        
    def target_callback(self, msg: Int32):
        """Callback for set position."""
        # Target Positions
        # targetposition1 = 100000
        # targetposition2 = 300000
        # targetposition3 = 500000
        # targetposition4 = 0

        # Extract target position from the received message
        # target_position = msg.data
    
        # Check if the motor is already moving towards a target
        # if not self.is_movement_finished:
        #     self.get_logger().warn("Motor is already moving. Ignoring new target.")
        #     return
    
        try:
            i = 0
            
            for i in range(len(self.target_positions)-1):
                actual_pos = self.c1.read_actual_pos()
                if not (actual_pos < self.target_positions[i] + 5 & actual_pos > self.target_positions[i] - 5): 
                    positionx = self.publish_position(i)
                    self.move_to_position(positionx)
                    time.sleep(5)

                else: 
                    self.get_logger().error(f"Error executing target: {self.target_positions[i]}")
    
            # After completing all target positions
            self.get_logger().info("All target positions reached.")
            print("Returning to PreOperational State")
            self.c1.NMT_Pre_Op()
            time.sleep(2)
    
            print("Entering Stop State")
            self.c1.NMT_Stop()
            time.sleep(2)
            # self.is_movement_finished = True
            time.sleep(5)
    
        except Exception as e:
            self.get_logger().error(f"Error executing target: {str(e)}")
            # self.is_movement_finished = True
            # self.get_logger().info("Executing target...")
            # self.is_movement_finished = False

            # # Your motor control logic here using the target_position
            # # Example: Move motor to target_position
            # # ENABLING DRIVER
            # self.c1.enable()

            # print(f"Setting Target position to {target_position}")
            # self.c1.set_target_location(target_position)
            # time.sleep(2)

            # # Absolute Position Start Trigger
            # print(f"Moving to position {target_position}")
            # self.c1.start_trigger_absolute()

            # self.monitor_position(self.c1, threshold=5, interval=0.2)


            # # DISABLING DRIVER
            # time.sleep(2)
            # self.c1.disable()
            # time.sleep(2)

            # # For this example, let's assume a simple logic of moving to target positions one by one
            # # More sophisticated logic can be implemented based on your motor control requirements

            # # Move to the next target position in the list
            # self.move_to_next_target()

    # def move_to_next_target(self):
        # if self.i < len(self.target_positions):
        #     target_position = self.target_positions[self.i]
        #     self.move_to_position(target_position)
        #     self.i += 1
        # else:
        #     self.get_logger().info("All target positions reached.")
        #     print("Returning to PreOperational State")
        #     self.c1.NMT_Pre_Op()
        #     time.sleep(2)

            # print("Entering Stop State")
            # self.c1.NMT_Stop()
            # time.sleep(2)
            # self.is_movement_finished = True


    def move_to_position(self, target_position):
        try:
            # Your motor control logic to move to the specified target_position
            # # == POSITION 2 == #
            # RE-ENABLE DRIVER
            self.c1.enable()

            # Move to target position 2
            # print(f"Setting Target position to {target_position}")
            # Simulating movement for demonstration
            self.get_logger().info(f"Setting Target position to {target_position}")
            self.c1.set_target_location(target_position)
            time.sleep(2)

            # Absolute Position Start Trigger
            print(f"Moving to position {target_position}")
            self.c1.start_trigger_absolute()

            # Continuous Positon Feedback

            self.monitor_position(self.c1, threshold=5, interval=0.2)

            # DISABLING DRIVER
            time.sleep(2)
            self.c1.disable()
            self.get_logger().info("Target position reached.")
            time.sleep(2)

            
            # time.sleep(2)  # Simulating movement time
            # self.is_movement_finished = True

        except Exception as e:
            self.get_logger().error(f"Error moving to position {target_position}: {str(e)}")
            # self.is_movement_finished = True


    def publish_position(self, n):
        try:
            msg = Int32()
            msg.data = self.target_positions[n]
            self.publisher_.publish(msg)

            # Read current position from your motor controller
            # actual_pos = self.monitor_position(instance, threshold=5, interval=0.2)

            # # Only publish the first target position at the start
            # if self.is_movement_finished and actual_pos is not None:
            #     self.get_logger().info("Publishing the first target position")
            #     msg = Int32()
            #     msg.data = self.target_positions[0]
            #     self.publisher_.publish(msg)
            #     self.is_movement_finished = False
            # else:
            #     # Check if the motor is within the target position range
            #     target_position = self.target_positions[self.i]
            #     if target_position - 10 <= actual_pos <= target_position + 10:
            #         self.get_logger().info(f"Target position {self.i} reached")
            #         time.sleep(5)  # Wait for a moment
            #         self.i += 1  # Move to the next target position
            #         if self.i < len(self.target_positions)-1:
            #             msg = Int32()
            #             msg.data = self.target_positions[self.i]
            #             self.publisher_.publish(msg)
            #             self.is_movement_finished = False
            #         else:
            #             self.get_logger().info("All target positions reached.")
            #             print("Returning to PreOperational State")
            #             self.c1.NMT_Pre_Op()
            #             time.sleep(2)
            #             print("Entering Stop State")
            #             self.c1.NMT_Stop()
            #             time.sleep(2)
            #             self.is_movement_finished = True
            #             self.i = 0  # Reset index for future use

        except Exception as e:
            self.get_logger().error(f"Error publishing position: {str(e)}")

        # try:


        #     self.get_logger().info("Executing target...")

            

            # READING CURRENT POSITION
            # actual_pos = self.c1.read_actual_pos()
            # print("Current Position = ", actual_pos)

            # feedback_msg = SetPosition.Feedback()
            # result = SetPosition.Result()
            # result.success = True

        # except canopen.sdo.SdoCommunicationError:
        #     print("Failed to update node timeout protection")
    
    def monitor_position(self, instance, threshold=5, interval=0.2):
        """
        Monitors the position of a servo motor continuously until the change
        in position is less than the specified threshold.

        Args:
        - c1: The DMKEServoDriver2_V1 object for the servo motor.
        - threshold (optional): The threshold for considering the change in
                                position significant. Defaults to 5.
        - interval (optional): The time interval (in seconds) between position
                               readings. Defaults to 0.2 seconds.
        """
        prev_pos = instance.read_actual_pos()
        while True:
            time.sleep(interval)
            actual_pos = instance.read_actual_pos()

            if actual_pos is not None and prev_pos is not None:
                print(f"Actual position: {actual_pos}")
                # self.get_logger().info(f"Actual position: {actual_pos}")

                # Check if position has changed significantly
                if abs(actual_pos - prev_pos) < threshold:
                    break
                
            prev_pos = actual_pos
            return actual_pos

    def destroy(self):
        # Clean up CANopen network
        self.c1.disable()
        self.network.disconnect()
        super().destroy_node()


def main(args=None):
    """Run when this script is called."""
    rclpy.init(args=args)

    node = ClampNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy()
    rclpy.shutdown()


    print("Task completed successfully")
    # network.disconnect()


if __name__ == '__main__':
    main()