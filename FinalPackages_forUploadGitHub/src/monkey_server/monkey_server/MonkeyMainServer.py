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

x = 0
n = 0
y = 0


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
                
def save_integer_to_file(number, file_path):
        if not isinstance(number, int):
            raise ValueError("Input must be an integer.")
            # print("Input not integer...Saving as 0")

        with open(file_path, 'w') as file:
            file.write(str(number))

def read_integer_from_file(file_path):
    # default_value = 0
    try:
        with open(file_path, 'r') as file:
            data = file.read().strip()
            return int(data)
    except (ValueError, FileNotFoundError):
        # If file not found, create the file with default value
        with open(file_path, 'w') as file:
            file.write(str(0))
        return 0
    
def dmke_save_filepath(node_id):
    if node_id == 2:
        position_data_filepath = 'dmke_UC_pos.txt'
    elif node_id == 3:
        position_data_filepath = 'dmke_LC_pos.txt'

    return position_data_filepath

def checkNoS():
    nx = read_integer_from_file("NumofStepsUP.txt")
    ny = read_integer_from_file("NumofStepsDOWN.txt")
    
    if nx != 0:
        nx -= 1
        save_integer_to_file(nx, "NumofStepsUP.txt")
    elif nx == 0:
        if ny != 0:
            ny -= 1
            save_integer_to_file(ny, "NumofStepsDOWN.txt")
        elif ny == 0:
            nx = 0
            ny = 0
            save_integer_to_file(nx, "NumofStepsUP.txt")
            save_integer_to_file(ny, "NumofStepsDOWN.txt")
        




##### ========== ---------- DMKE ----------  ========== ####

# SERVICE #
class DMKECheckPositionService(Node):

    def __init__(self):
        super().__init__('check_position_service')
        self.srv = self.create_service(GetPosition, 'get_position', self.get_position_callback)        # CHANGE
        self.get_logger().info('GetPosition Server is ready!')


    def get_position_callback(self, request, response):
        serv_node_id = request.node_id
        file_to_read = dmke_save_filepath(serv_node_id)
        response.position = read_integer_from_file(file_to_read)                                    # CHANGE
        # self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response

# ACTION SERVER #
class DMKESetPositionServer(Node):

    def __init__(self):
        super().__init__('dmke_set_position_action_server')
        self._action_server = ActionServer(
            self,
            SetPosition,
            'set_position',
            self.execute_callback,
            cancel_callback=self.cancel_callback
            )

        self.network = canopen.Network()
        self.network.connect(interface='seeedstudio', 
                             channel='/dev/ttyUSB0', 
                             baudrate=115200, 
                             bitrate=500000)

        time.sleep(1)
        self.uc = DMKEServoDriver(self.network, 2)
        # self.lc = DMKEServoDriver(self.network, 3)

        # self.uc.NMT_Reset_Node()
        self.uc.NMT_Reset_Comm()

        self.uc.NMT_Pre_Op()
        self.uc.NMT_Start()

        time.sleep(1)

        # self.lc.NMT_Reset_Node()
        # self.lc.NMT_Reset_Comm()

        # self.lc.NMT_Pre_Op()
        # self.lc.NMT_Start()
        time.sleep(1)

        self.uc.enable()
        # self.lc.enable()

        print("Setting positional control mode")
        self.uc.set_pos_control_mode()
        # self.lc.set_pos_control_mode()
        time.sleep(2)

        print("Setting Parameters for position control mode")
        self.uc.set_profile_velocity(2000)
        self.uc.set_profile_acceleration(10000)
        self.uc.set_profile_deceleration(10000)

        # self.lc.set_profile_velocity(2000)
        # self.lc.set_profile_acceleration(10000)
        # self.lc.set_profile_deceleration(10000)
        time.sleep(2)
            

        self.get_logger().info('DMKE SERVERS INITIALIZED')

        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal: Move motor to position %d' % goal_handle.request.target_position)
        
        action_node_id = goal_handle.request.node_id
        target_position = goal_handle.request.target_position

        # self.network = canopen.Network()
        # self.network.connect(interface='seeedstudio', 
        #                      channel='/dev/ttyS0', 
        #                      baudrate=115200, 
        #                      bitrate=500000)
        filepathx = dmke_save_filepath(action_node_id)
        self.motor = DMKEServoDriver(self.network, action_node_id)

        # Move to target position
        real_target1 = self.real_pos(self.motor, target_position, filepathx)

        print(f"Setting Target position to {target_position}")
        
        self.motor.set_target_location(real_target1)
        time.sleep(2)

        print(f"Moving to position {target_position}")
        # print(f"Encoder Counts Left: {real_target1}")
        self.motor.start_trigger_absolute()

        # real_pos = read_integer_from_file('dmke_encoder_pos.txt')
        # pospos = real_pos + c1.read_actual_pos()
        # save_integer_to_file(pospos, 'dmke_encoder_pos.txt')
        # input = check_pos(c1, 'dmke_encoder_pos.txt')
        
        self.monitor_position(goal_handle, self.motor, filepathx)
        # count += 1

        time.sleep(2)
        self.motor.disable
        time.sleep(2)

        # self.c1.NMT_Pre_Op()
        # time.sleep(2)

        data_from_file = read_integer_from_file(filepathx)
        read_current = self.motor.read_actual_current()
        
        # Determine the success of the action
        if abs(target_position - data_from_file) <= 5000 or read_current >= 4500:
            dmke_condition = True
        # success = abs(target_position - data_from_file) <= 5000

        # Create the result message
        result = SetPosition.Result()
        
        result.success_pos = dmke_condition

        if result.success_pos:
            self.get_logger().info('Motor reached target position')
            goal_handle.succeed()
            
            checkNoS()
            
                
        else:
            self.get_logger().info('Motor failed to reach target position')
            goal_handle.abort()

        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal canceled: Move motor to position %d' % goal_handle.request.target_position)
        goal_handle.canceled()

    def real_pos(self, instance, target_pos, file_path):
        
        starting_pos = read_integer_from_file(file_path)
        read_pos = instance.read_actual_pos()

        if starting_pos - 5000 <= read_pos <= starting_pos + 5000:
            starting_pos = read_pos

        if not (starting_pos - 5000 <= read_pos <= starting_pos + 5000):
            starting_pos = starting_pos

        if read_pos is None:
            starting_pos = starting_pos

        print(f"Current Position: {starting_pos}")
        real_target = read_pos + (target_pos - starting_pos)
        return real_target
    
    # def save_integer_to_file(number, file_path):
    #     if not isinstance(number, int):
    #         raise ValueError("Input must be an integer.")
    #         # print("Input not integer...Saving as 0")

    #     with open(file_path, 'w') as file:
    #         file.write(str(number))

    # def read_integer_from_file(file_path):
    #     # default_value = 0
    #     try:
    #         with open(file_path, 'r') as file:
    #             data = file.read().strip()
    #             return int(data)
    #     except (ValueError, FileNotFoundError):
    #         # If file not found, create the file with default value
    #         with open(file_path, 'w') as file:
    #             file.write(str(0))
    #         return 0

    def monitor_position(self, goal_handle, instance, filepath, threshold=50, interval=0.1):
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
        feedback_msg = SetPosition.Feedback()

        while True:
            time.sleep(interval)
            try:
                # Read the actual position
                actual_pos = instance.read_actual_pos()

                # Handle the case where actual_pos is a valid position
                if actual_pos is not None and prev_pos is not None:

                    if actual_pos < prev_pos:
                        sub_pos = abs((prev_pos - actual_pos))
                        file_pos =read_integer_from_file(filepath) - sub_pos
                        print(f"Actual position: {file_pos}")
                        # save_integer_to_file(file_pos, filepath)
                        feedback_msg.current_position = file_pos
                        save_integer_to_file(file_pos, filepath)

                    elif actual_pos > prev_pos:
                        sub_pos = abs((prev_pos - actual_pos))
                        file_pos = read_integer_from_file(filepath) + sub_pos
                        print(f"Actual position: {file_pos}")
                        feedback_msg.current_position = file_pos
                        save_integer_to_file(file_pos, filepath)

                    goal_handle.publish_feedback(feedback_msg)

                    # Check if position has changed significantly
                    if abs(actual_pos - prev_pos) < threshold:
                        # read_pos = instance.read_actual_pos()
                        x = abs(actual_pos - prev_pos)
                        if actual_pos < prev_pos:
                            file_pos = read_integer_from_file(filepath) - x
                            save_integer_to_file(file_pos, filepath)
                        elif actual_pos > prev_pos:
                            file_pos = read_integer_from_file(filepath) + x
                            save_integer_to_file(file_pos, filepath)
                        
                        print(actual_pos)
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
                    # self.monitor_position(self, goal_handle, instance, filepath, threshold=3, interval=0.05)
                    continue




##### ========== ---------- CYLINDER ----------  ========== ####
class CylinderSetExtension(Node):
    """MonKey Arm cylinder control wrapper for IDSServoDriver."""

    def __init__(self):
        """MonKey Arm cylinder control node."""
        super().__init__('cylinder_action_server')

        # Declare action server
        # self.action_server = ActionServer(self, SetExtension, 'SetExtension', self.action_callback)

        # # Start message
        # self.get_logger().info(f"SetExtension Server started")

        # self.declare_parameter('can_id', 1,
        # ParameterDescriptor(description='CAN ID of the target driver.'))
        # self.declare_parameter('can_channel', '/dev/ttyS0',
        #     ParameterDescriptor(description='Channel of CAN tranceiver.'))
        # self.declare_parameter('can_baudrate', 2000000,
        #     ParameterDescriptor(description='Baudrate of USB communication.'))
        # self.declare_parameter('can_bitrate', 500000,
        #     ParameterDescriptor(description='Bitrate of CAN communication bus in bit/s.'))

        # # Get parameters
        # self.can_id = self.get_parameter('can_id').value
        # can_channel = self.get_parameter('can_channel').value
        # baudrate = self.get_parameter('can_baudrate').value
        # bitrate = self.get_parameter('can_bitrate').value

        # # Create CAN connection
        # bus = can.ThreadSafeBus(
        #     interface='seeedstudio', channel=can_channel, baudrate=baudrate, bitrate=bitrate
        #     # interface='socketcan', channel='can0', bitrate=500000
        # )

        # time.sleep(0.5)

        # # Initialize driver
        # self.cylinder = IDSServoDriver(bus, self.can_id, name="Cylinder")
        # self.cylinder.fault_reset()

        # time.sleep(0.5)
        # self.cylinder.set_positional_control_mode()

        # self.get_logger().info('CYLINDER INITIALIZED')


    def action_callback(self, goal_handle:ServerGoalHandle, filepath='saved_extension.txt', interval=0.1, threshold=0.05):
        """Callback for set extension action."""
        self.get_logger().info("Executing goal...")

        # self.driver.set_positional_control_mode()
        
        self.driver.set_extension(goal_handle.request.target_extension, 8, False)
        # actual_ext = self.driver.extension_feedback()
        
        self.monitor_extension(goal_handle, self.driver, filepath, message=self.bus.recv())  

        checkNoS()

        # self.driver.disable()

        result = SetExtension.Result()

        data_from_file2 = read_value_from_file('saved_extension.txt')
        success2 = abs(goal_handle.request.target_extension - data_from_file2) <= 0.1

        # Create the result message
        result = SetPosition.Result()
        
        result.success_ext = success2

        if success2:
            self.get_logger().info('Motor reached target position')
            goal_handle.succeed()
            
            checkNoS()
            
        else:
            self.get_logger().info('Motor failed to reach target position')
            goal_handle.abort()

        return result

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
        response.extensiongot = read_value_from_file('saved_extension.txt')                                           # CHANGE
        # self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response
    
    


def main(args=None):
    rclpy.init(args=args)
    # motor_init_server = MotorInit()
    DMKE_set_pos = DMKESetPositionServer()
    DMKE_get_pos = DMKECheckPositionService()
    cylinder_get_ext = CheckExtensionService()
    cylinder_set_ext = CylinderSetExtension()

    try:
        rclpy.spin(DMKE_set_pos)
        rclpy.spin(DMKE_get_pos)
        rclpy.spin(cylinder_set_ext)
        rclpy.spin(cylinder_get_ext)

        # motor_init_server.destroy_node()
        DMKE_set_pos.destroy_node()
        DMKE_get_pos.destroy_node()
        cylinder_set_ext.destroy_node()
        cylinder_get_ext.destroy_node()

    # network.disconnect()
    except KeyboardInterrupt:

        rclpy.shutdown()

if __name__ == '__main__':
    main()
