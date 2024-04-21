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
        position_data_filepath = "dmke_UC_pos.txt"
    elif node_id == 3:
        position_data_filepath = "dmke_LC_pos.txt"

    return position_data_filepath

# def checkNoS():
#     nx = read_integer_from_file("NumofStepsUP.txt")
#     ny = read_integer_from_file("NumofStepsDOWN.txt")
    
#     if nx != 0:
#         nx -= 1
#         save_integer_to_file(nx, "NumofStepsUP.txt")
#     elif nx == 0:
#         if ny != 0:
#             ny -= 1
#             save_integer_to_file(ny, "NumofStepsDOWN.txt")
#         elif ny == 0:
#             nx = 0
#             ny = 0
#             save_integer_to_file(nx, "NumofStepsUP.txt")
#             save_integer_to_file(ny, "NumofStepsDOWN.txt")
#             print("SUCCESSFUL EXECUTION OF BT, NO PLS OCCURRED")
        


##### ==================== ---------- DMKE ----------  ==================== ####

class DMKEServers(Node):

    def __init__(self):
        super().__init__('DMKE_SERVERS')
        self.dmke_action_serveer = ActionServer(
            self,
            SetPosition,
            'set_position',
            self.execute_callback,
            # cancel_callback=self.cancel_callback
            )
        
        self.dmke_srv = self.create_service(GetPosition, 'get_position', self.get_position_callback)        # CHANGE
        # self.get_logger().info('GetPosition Server is ready!')

        self.network = canopen.Network()
        self.network.connect(interface='seeedstudio', 
                             channel='/dev/ttyUSB2', 
                             baudrate=115200, 
                             bitrate=500000)

        time.sleep(0.5)
        self.uc = DMKEServoDriver(self.network, 2)
        self.lc = DMKEServoDriver(self.network, 3)

        # self.uc.NMT_Reset_Node()
        self.uc.NMT_Reset_Comm()

        # self.lc.NMT_Reset_Node()
        self.lc.NMT_Reset_Comm()

        self.uc.NMT_Pre_Op()
        self.uc.NMT_Start()

        time.sleep(0.5)

        self.lc.NMT_Pre_Op()
        self.lc.NMT_Start()
        time.sleep(0.5)

        self.uc.enable()
        self.lc.enable()

        print("Setting positional control mode")
        self.uc.set_pos_control_mode()
        self.lc.set_pos_control_mode()
        time.sleep(0.5)

        print("Setting Parameters for position control mode")
        self.uc.set_profile_velocity(2500)
        self.uc.set_profile_acceleration(15000)
        self.uc.set_profile_deceleration(15000)

        self.lc.set_profile_velocity(2500)
        self.lc.set_profile_acceleration(15000)
        self.lc.set_profile_deceleration(15000)
        time.sleep(0.5)
            

        self.get_logger().info('>>>>>>>>>> DMKE SERVERS INITIALIZED <<<<<<<<<<')

        
    def execute_callback(self, goal_handle):
        
        action_node_id = goal_handle.request.node_id
        print(f"Action Node ID: {action_node_id}")
        
        target_position = goal_handle.request.target_position

        filepathx = dmke_save_filepath(action_node_id)
        self.motor = DMKEServoDriver(self.network, action_node_id)
        
        if target_position > 650000 or target_position < -20000:
            self.motor.disable()
            self.get_logger().info('Received goal exceeded range of operation!! : %d ' % target_position)
            result = SetPosition.Result()
            result.success_pos = False
            goal_handle.abort()
            return result

        else:
            print(f"Setting Target position to {target_position}")
            self.get_logger().info('Received goal: Moving motor to position %d' % target_position)

            # Move to target position
            real_target1 = self.real_pos(self.motor, target_position, filepathx)

            self.motor.enable()
            self.motor.set_target_location(real_target1)
            # time.sleep()

            # print(f"Moving to position {target_position}")
            # print(f"Encoder Counts Left: {real_target1}")
            self.motor.start_trigger_absolute()

            self.monitor_position(goal_handle, self.motor, filepathx)

            self.motor.disable
            time.sleep(0.5)

            data_from_file = read_integer_from_file(filepathx)
            current_from_file = read_integer_from_file('MaxCurrent.txt')

            # Determine the success of the action
            if abs(target_position - data_from_file) <= 10000 or current_from_file > 5000:
                dmke_condition = True
                save_integer_to_file(0, 'MaxCurrent.txt')
            else:
                dmke_condition = False
            # success = abs(target_position - data_from_file) <= 5000

            # Create the result message
            result = SetPosition.Result()

            result.success_pos = dmke_condition

            if result.success_pos:
                self.get_logger().info('Motor reached target position')
                goal_handle.succeed()

                # checkNoS()

            else:
                self.get_logger().info('Motor failed to reach target position')
                goal_handle.abort()

            return result

    # def cancel_callback(self, goal_handle):
    #     self.get_logger().info('Goal canceled: Move motor to position %d' % goal_handle.request.target_position)
    #     goal_handle.canceled()


    def get_position_callback(self, request, response):
        serv_node_id = request.node_id
        file_to_read = dmke_save_filepath(serv_node_id)
        response.positiongot = read_integer_from_file(file_to_read)

        return response

    def real_pos(self, instance, target_pos, file_path):
        
        starting_pos = read_integer_from_file(file_path)
        read_pos = instance.read_actual_pos()

        if starting_pos - 2000 <= read_pos <= starting_pos + 2000:
            starting_pos = read_pos

        if not (starting_pos - 2000 <= read_pos <= starting_pos + 2000):
            starting_pos = starting_pos

        if read_pos is None:
            starting_pos = starting_pos

        print(f"Current Position: {starting_pos}")
        real_target = read_pos + (target_pos - starting_pos)
        return real_target
    
    
    def monitor_position(self, goal_handle, instance, filepath, threshold=50, interval=0.05):
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
        prev_current = instance.read_actual_current()
        # print(f"Position from Motor's Perspective {prev_pos}")
        error_count = 0  # Initialize error count
        feedback_msg = SetPosition.Feedback()

        while True:
            time.sleep(interval)
            try:
                # Read the actual position
                actual_pos = instance.read_actual_pos()

                # Read the actual current
                read_current = instance.read_actual_current()
                print(f"Actual Current: {read_current}")

                abs_read_current = abs(read_current)
                abs_prev_current = abs(prev_current)

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
                        
                    goal_handle.publish_feedback(feedback_msg)
                    prev_pos = actual_pos
                    

                if read_current is not None and prev_current is not None:
                    if abs_read_current >= abs_prev_current:
                        max_current = abs_read_current
                        save_integer_to_file(max_current, 'MaxCurrent.txt')

                    if abs_prev_current >= abs_read_current:
                        max_current = abs_prev_current
                        save_integer_to_file(max_current, 'MaxCurrent.txt')

                    if abs_read_current > 5000 or read_integer_from_file('MaxCurrent.txt') > 5000:
                        self.get_logger().info('Current Preset Threshold Limit Reached: %f' % read_current)
                        x = abs(actual_pos - prev_pos)
                        if actual_pos < prev_pos:
                            file_pos = read_integer_from_file(filepath) - x
                            save_integer_to_file(file_pos, filepath)
                            
                        elif actual_pos > prev_pos:
                            file_pos = read_integer_from_file(filepath) + x
                            save_integer_to_file(file_pos, filepath)
                        instance.disable()
                        break
                    
                    prev_current = read_current

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





##### ==================== ---------- CYLINDER ----------  ==================== ####


class CylinderServers(Node):
    """MonKey Arm cylinder control wrapper for IDSServoDriver."""

    def __init__(self):
        """MonKey Arm cylinder control node."""
        super().__init__('CylinderServers')

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
        self.driver = IDSServoDriver(self.bus, self.can_id, name="Linear Actuator")
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

        if real_target_ext > 1.3 or real_target_ext < 0.0:
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
    # motor_init_server = MotorInit()
    DMKE_servers = DMKEServers()
    # Cylinder_servers = CylinderServers()

    try:
        rclpy.spin(DMKE_servers)
        # rclpy.spin(Cylinder_servers)

    # network.disconnect()
    except KeyboardInterrupt:
        DMKE_servers.destroy_node()
        # Cylinder_servers.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





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

##### ========== ---------- DMKE ----------  ========== ####

# # SERVICE #
# class DMKECheckPositionService(Node):

#     def __init__(self):
#         super().__init__('check_position_service')
#         self.srv = self.create_service(GetPosition, 'get_position', self.get_position_callback)        # CHANGE
#         self.get_logger().info('GetPosition Server is ready!')


#     def get_position_callback(self, request, response):
#         serv_node_id = request.node_id
#         file_to_read = dmke_save_filepath(serv_node_id)
#         response.positiongot = read_integer_from_file(file_to_read)                                    # CHANGE
#         # self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

#         return response

        
# SERVICE #
# class CheckExtensionService(Node):

#     def __init__(self):
#         super().__init__('check_extension_service')
#         self.srv = self.create_service(GetExtension, 'GetExtension', self.get_extension_callback)        # CHANGE
#         self.get_logger().info('GetExtension Server is ready!')


#     def get_extension_callback(self, response):
#         # node_id = request.node_id.to_bytes(1, byteorder='little')
#         response.extensiongot = read_value_from_file('saved_extension.txt')                                           # CHANGE
#         # self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

#         return response



 # DMKE_get_pos = DMKECheckPositionService()
    # cylinder_get_ext = CheckExtensionService()
    # cylinder_set_ext = CylinderSetExtension()


    # rclpy.spin(DMKE_get_pos)
# rclpy.spin(cylinder_set_ext)
# rclpy.spin(cylinder_get_ext)
# motor_init_server.destroy_node()
# DMKE_set_pos.destroy_node()
# DMKE_get_pos.destroy_node()
# cylinder_set_ext.destroy_node()
# cylinder_get_ext.destroy_node()