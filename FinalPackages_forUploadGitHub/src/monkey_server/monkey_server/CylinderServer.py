import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time
import canopen
import can
from monkey_interface.action import SetPosition
from monkey_interface.action import SetExtension
from monkey_interface.srv import GetPosition
from monkey_interface.srv import GetExtension
# from monkey_interface.srv import MotorInit
from monkey_server.DMKEServoDriver import DMKEServoDriver
from rcl_interfaces.msg import ParameterDescriptor
from monkey_server.IDSServoDriver import IDSServoDriver
from rclpy.action.server import ServerGoalHandle



def save_value_to_file(number, file_path):
    if not isinstance(number, float):
        raise ValueError("Input must be a float.")
        # print("Input not integer...Saving as 0")
    with open(file_path, 'w') as file:
        file.write(str(number))

def read_value_from_file(file_path):
    # default_value = 0
    try:
        with open(file_path, 'r') as file:
            data = file.read().strip()
            return float(data)
    
    except (ValueError, FileNotFoundError):
        # If file not found, create the file with default value
        with open(file_path, 'w') as file:
            file.write(str(0))
        return 0
    

##### ==================== ---------- CYLINDER ----------  ==================== ####


class CylinderServers(Node):
    """MonKey Arm cylinder control wrapper for IDSServoDriver."""

    def __init__(self):
        """MonKey Arm cylinder control node."""
        super().__init__('CylinderServers')

        self.declare_parameter('can_id', 1,
            ParameterDescriptor(description='CAN ID of the target driver.'))
        self.declare_parameter('can_channel', '/dev/ttyUSB1',
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
            # interface='seeedstudio', channel=can_channel, baudrate=baudrate, bitrate=bitrate
            interface='socketcan', channel='can0', baudrate=baudrate, bitrate=500000,
        )

        time.sleep(0.5)

        # Initialize driver
        self.driver = IDSServoDriver(self.bus, can_id=1, name="Linear Actuator")
        self.get_logger().info("Cylinder driver initialized with name 'Linear Actuator' ")

        self.driver.fault_reset()
        time.sleep(0.5)
        self.driver.set_positional_control_mode()

        # Declare Action server
        self.action_server = ActionServer(self, SetExtension, 'SetExtension', self.action_callback)

        # Declare Service server
        self.service_server = self.create_service(GetExtension, 'GetExtension', self.get_extension_callback)
        # self.get_logger().info('GetExtension Server is ready!')

        # Start message
        self.get_logger().info(">>>>>>>>>> CYLINDER SERVERS INITIALIZED <<<<<<<<<<")

    def action_callback(self, goal_handle, filepath='saved_extension.txt'):
        """Callback for set extension action."""
        
        target_extension = goal_handle.request.target_extension

        if target_extension < 0 or target_extension > 1.2:
            goal_handle.abort()
            result = SetExtension.Result()
            result.success_ext = False
            self.get_logger().info("Extension Value Set Exceeded Expected Range")
            return result
        
        else:
            self.get_logger().info("Executing goal...")
            real_target = self.real_ext(target_extension, filepath)
          
            self.driver.set_extension(real_target, 10, False)

            self.monitor_extension(goal_handle, filepath)

            self.driver.disable()

            data_from_file = read_value_from_file(filepath)
            
            # Determine the success of the action
            if abs(target_extension - data_from_file) <= 0.005:
                cylinder_condition = True

                # checkNoS()
            else:
                cylinder_condition = False
        
            # Create the result message
            result = SetExtension.Result()

            result.success_ext = cylinder_condition

            if result.success_ext:
                self.get_logger().info('Cylinder reached target extension')
                goal_handle.succeed()

            else:
                self.get_logger().info('Cylinder failed to reach target extension')
                goal_handle.abort()

            return result

    
    def get_extension_callback(self, request, response):
        # node_id = request.node_id.to_bytes(1, byteorder='little')
        response.extensiongot = read_value_from_file('saved_extension.txt')
       
        return response

    def real_ext(self, target_ext, file_path):
        starting_ext = read_value_from_file(file_path)
        read_ext = self.driver.extension

        if (starting_ext - 0.002) <= read_ext <= (starting_ext + 0.002):
            starting_ext2 = read_ext

        if not (starting_ext - 0.002 <= read_ext <= starting_ext + 0.002):
            starting_ext2 = starting_ext

        if read_ext is None:
            starting_ext2 = starting_ext

        if starting_ext2 < 0.0:
            self.driver.disable()
            print("Starting Extension Value is < 0, Stopping Operation...")

        print(f"Current Extension: {starting_ext2}")
        real_target_ext = read_ext + (target_ext - starting_ext2)

        if real_target_ext > 1.2 or real_target_ext < -1.2:
            self.driver.disable()
            print(f"Real Target Extension Over Limit: {real_target_ext}")
            return None
        
        else:
            print(f"TARGET EXTENSION DIFFERENCE = {real_target_ext}")
            return real_target_ext


    def monitor_extension(self, goal_handle, filepath, interval = 0.05):
        """
        Monitors the extension of a linear actuator continuously until the change
        in extension is less than the specified threshold.

        Args:
        - interval (optional): The time interval (in seconds) between extension
                               readings. Defaults to 0.01 seconds.
        """

        # prev_ext = self.driver.on_message_received(msg=can.Message)
        prev_ext = self.driver.extension
        # print(f"extension from Motor's Perspective {prev_ext}")
        error_count = 0  # Initialize error count
        count = 0
        feedback_msg = SetExtension.Feedback()

        while self.driver.is_running:
            
            time.sleep(interval)
            try:
                # Read the actual extension
                # actual_ext = self.driver.on_message_received(msg=can.Message)
                actual_ext = self.driver.extension

                # Handle the case where actual_ext is a valid extension
                if actual_ext is not None and prev_ext is not None:

                    if actual_ext < prev_ext:
                        sub_ext = abs((prev_ext - actual_ext))
                        file_ext = read_value_from_file(filepath) - sub_ext
                        print(f"Actual extension: {file_ext}")
                        # save_value_to_file(file_ext, filepath)
                        feedback_msg.current_extension = file_ext
                        save_value_to_file(file_ext, filepath)

                    elif actual_ext > prev_ext:
                        sub_ext = abs((prev_ext - actual_ext))
                        # sub_ext = abs((prev_ext - actual_ext))
                        file_ext = read_value_from_file(filepath) + sub_ext
                        print(f"Actual extension: {file_ext}")
                        feedback_msg.current_extension = file_ext
                        save_value_to_file(file_ext, filepath)

                    if actual_ext == prev_ext:
                        count += 1
                        file_ext = read_value_from_file(filepath)
                        print(f"Actual extension: {file_ext}")
                        feedback_msg.current_extension = file_ext
                        save_value_to_file(file_ext, filepath)

                    if count > 60:
                        self.driver.disable()
                        print("..........EXTENSION FAILED..........")
                        break
                    
                    # self.driver.disable()
                    goal_handle.publish_feedback(feedback_msg)
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



def main(args=None):
    rclpy.init(args=args)

    # DMKE_servers = DMKEServers()
    Cylinder_servers = CylinderServers()

    try:
        # rclpy.spin(DMKE_servers)
        rclpy.spin(Cylinder_servers)

    # network.disconnect()
    except KeyboardInterrupt:
        # DMKE_servers.destroy_node()
        Cylinder_servers.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
