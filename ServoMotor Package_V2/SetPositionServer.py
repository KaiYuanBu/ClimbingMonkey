import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time
import canopen
from dmke_interface.action import SetPosition
from dmke_package.DMKEServoDriver2 import DMKEServoDriver2_V1



class SetPositionActionServer(Node):

    def __init__(self):
        super().__init__('set_position_action_server')
        self._action_server = ActionServer(
            self,
            SetPosition,
            'set_position',
            self.execute_callback,
            cancel_callback=self.cancel_callback
            )
        
        node_id =  0x01
        self.network = canopen.Network()
        self.network.connect(interface='seeedstudio', 
                             channel='/dev/ttyS0', 
                             baudrate=115200, 
                             bitrate=500000)
        # '/dev/ttyS0'
        time.sleep(1)
        self.c1 = DMKEServoDriver2_V1(self.network, node_id)
        self.c1.NMT_Reset_Node()
        self.c1.NMT_Reset_Comm()
        
        self.c1.NMT_Pre_Op()
        time.sleep(2)
        
        self.c1.NMT_Start()
        time.sleep(2)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal: Move motor to position %d' % goal_handle.request.target_position)
        
        target_position = goal_handle.request.target_position

        self.c1.enable()

        print("Setting positional control mode")
        self.c1.set_pos_control_mode()
        time.sleep(2)

        print("Setting Parameters for position control mode")
        self.c1.set_profile_velocity(3000)
        self.c1.set_profile_acceleration(1000)
        self.c1.set_profile_deceleration(1000)
        time.sleep(2)

        # Move to target position 1
        real_target1 = self.real_pos(self.c1, target_position, 'dmke_encoder_pos.txt')

        print(f"Setting Target position to {target_position}")
        
        self.c1.set_target_location(real_target1)
        time.sleep(2)

        print(f"Moving to position {target_position}")
        print(f"Encoder Counts Left: {real_target1}")
        self.c1.start_trigger_absolute()

        self.monitor_position(goal_handle, self.c1, 'dmke_encoder_pos.txt', threshold=3, interval=0.05)

        time.sleep(2)
        self.c1.disable
        time.sleep(2)

        self.c1.NMT_Pre_Op()
        time.sleep(2)

        data_from_file = self.read_integer_from_file(file_path='dmke_encoder_pos.txt')
        # Determine the success of the action
        success = abs(target_position - data_from_file) <= 5

        # Create the result message
        result = SetPosition.Result()
        
        result.success = success

        if result.success:
            self.get_logger().info('Motor reached target position')
            goal_handle.succeed()
        else:
            self.get_logger().info('Motor failed to reach target position')
            goal_handle.abort()

        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal canceled: Move motor to position %d' % goal_handle.request.target_position)
        goal_handle.canceled()

    

    def check_pos(self, instance, file_path):
        data1 = instance.read_actual_pos()
        data2 = self.read_integer_from_file(file_path)

        if data2 - 2 <= data1 <= data2 + 2:
            return data1

        if not (data2 - 2 <= data1 <= data2 + 2):
            return data2

        if data1 is None:
            return data2


    def save_integer_to_file(self, number, file_path):
        if not isinstance(number, int):
            raise ValueError("Input must be an integer.")
            # print("Input not integer...Saving as 0")

        with open(file_path, 'w') as file:
            file.write(str(number))

    def read_integer_from_file(self, file_path):
        # default_value = 0
        try:
            with open(file_path, 'r') as file:
                data = file.read().strip()
                return int(data)
        
        except FileNotFoundError:
            # If the file does not exist, create it and return 0
            self.get_logger().warn(f"File {file_path} not found. Creating it with default value 0.")
            self.save_integer_to_file(0, file_path)
            return 0
        # except (ValueError, FileNotFoundError):
        #     # If file not found, create the file with default value
        #     with open(file_path, 'w') as file:
        #         file.write(str(default_value))
        #     return default_value

    def real_pos(self, instance, target_pos, file_path):
        # starting_pos = self.read_integer_from_file(file_path)
        starting_pos = self.check_pos(instance, file_path)
        print(f"Current Position: {starting_pos}")
        real_target = target_pos - starting_pos
        return real_target


    def monitor_position(self, goal_handle, instance, filepath, threshold=3, interval=0.1):
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
        # print(f"Position from Motor's Perspective {prev_pos}")
        error_count = 0  # Initialize error count

        while True:
            time.sleep(interval)
            try:
                # Read the actual position
                actual_pos = instance.read_actual_pos()

                # Handle the case where actual_pos is a valid position
                if actual_pos is not None and prev_pos is not None:

                    if actual_pos < prev_pos:
                        sub_pos = abs((prev_pos - actual_pos))
                        file_pos =self.read_integer_from_file(filepath) - sub_pos
                        print(f"Actual position: {file_pos}")
                        # save_integer_to_file(file_pos, filepath)

                    elif actual_pos > prev_pos:
                        sub_pos = abs((prev_pos - actual_pos))
                        file_pos = self.read_integer_from_file(filepath) + sub_pos
                        print(f"Actual position: {file_pos}")

                    self.save_integer_to_file(file_pos, filepath)
                    feedback_msg = SetPosition.Feedback()
                    feedback_msg.current_position = file_pos
                    goal_handle.publish_feedback(feedback_msg)

                    # Check if position has changed significantly
                    if abs(actual_pos - prev_pos) < threshold:
                        read_pos = instance.read_actual_pos()
                        print(read_pos)
                        break

                    prev_pos = actual_pos

                else:
                    # raise ValueError("Failed to read current position of motor")
                    continue

            except canopen.SdoAbortedError as e:
                error_code = e.code
                print("SDO Aborted Error Code:", hex(error_code))
                error_count += 1  # Increase error count on each occurrence

                if error_count >= 5:
                    raise ValueError("SDO Aborted Error occurred 5 times. Stopping the function.")

                else:
                    continue



# feedback_msg = SetPosition.Feedback()
# feedback_msg.current_position = actual_pos
# goal_handle.publish_feedback(feedback_msg)
                

def main(args=None):
    rclpy.init(args=args)

    set_position_server = SetPositionActionServer()

    rclpy.spin(set_position_server)

    set_position_server.destroy_node()

    # network.disconnect()

    rclpy.shutdown()

if __name__ == '__main__':
    main()