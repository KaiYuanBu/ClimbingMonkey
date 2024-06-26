"""
Python driver class for IDS Servo Driver.

Written by:
    1. Kean Hao
Last Updated: 29 August 2023
"""

from enum import Enum

import threading
import time
import ctypes
import can

def read_value_from_file(file_path):
    # default_value = 0
    # try:
    with open(file_path, 'r') as file:
        data = file.read().strip()
        return float(data)

class FunctionCode(Enum):
    """Class for Function Code of CAN message in IDS Servo Driver."""

    # One to one write. Data is not stored when powered off.
    OTO_WRITE_SEND = 0x1A # Write to driver
    OTO_WRITE_RETURN = 0x1B # Return value from driver after writing
    OTO_WRITE_ERROR = 0x1C # Send when write to driver has error

    # One to one read.
    OTO_READ_SEND = 0x2A # Read request from driver
    OTO_READ_RETURN = 0x2B # Return value from driver after read request
    OTO_READ_ERROR = 0x2C # Send when read value from driver has error

    # Others
    REPORT = 0x88 # Received when report interval is set to > 0

    # One to many is not implemented.

class RegisterAddress(Enum):
    """Enum for register address of CAN message in IDS Servo Driver."""

    #------Write Address------
    # One to one write. Data is not stored when powered off.
    SERVO_START_STOP = 0x00 # Start/stop servo
    CONTROL_MODE = 0x02 # Control mode selection
    TARGET_SPEED = 0x06 # Target speed (RPM) = write_value / 8192 * 3000
    TARGET_FORCE = 0x08 # Target force in current (A)
    GROUP_ID = 0x0b # CAN group ID
    REPORT_INTERVAL = 0x0C # Status report interval in milliseconds
    CAN_ID = 0x0D # Servo driver's CAN ID
    REPORT_CONTENT = 0x2E # Status report content
    POSITIONAL_MODE_ACCELERATION_TIME = 0x09 # Acceleration / Decceleration time
    TARGET_POSITION_MSB = 0x50 # Set target position 16bit MSB
    TARGET_POSITION_LSB = 0x05 # Set target position 16bit LSB
    CURRENT_POSITION_MSB = 0x3C # Target position 16bit MSB
    CURRENT_POSITION_LSB = 0x3D # Target position 16bit LSB
    ERROR_RESET = 0x4A # Clear all error
    RATED_CURRENT = 0x2C # Rated current in mA
    OVERLOAD_FACTOR = 0x30 # Overload factor in % of rated current
    CLEAR_POSITION = 0x4C # Set current position to 0
    EMERGENCY_STOP = 0x4D # Stop all motion immediately
    SLOW_STOP = 0x4F # Stop all motion with decceleration
    TARGET_HOME = 0x53 # Search for Z signal (home)

    #------Read Address------
    READ_BUS_VOLTAGE = 0xE1 # Voltage of input bus in V
    READ_OUTPUT_CURRENT = 0xE2 # Current output in A
    READ_OUTPUT_SPEED = 0xE4 # Motor rotation speed = data / 8192 * 3000 RPM
    READ_TARGET_POSITION_MSB = 0xE6 # Read target position 16bit MSB
    READ_TARGET_POSITION_LSB = 0xE7 # Read target position 16bit LSB
    READ_CURRENT_POSITION_MSB = 0xE8 # Read current position 16bit MSB
    READ_CURRENT_POSITION_LSB = 0xE9 # Read current position 16bit LSB
    READ_ERROR_STATE = 0xE3 # Read the status of error

class ReportContentType(Enum):
    """Enum for report content type in IDS Servo Driver."""

    ALL = 0
    POSITION = 1
    ELECTRICAL = 2

class IDSServoDriver(can.listener.Listener):
    """Driver class for IDS Servo Driver. Used in Nenkeen electric cylinders."""

    # -----------Constants--------------
    ENCODER_CNT_PER_METRE = 1000000 # Encoder count per metre of extension

    def __init__(self, bus:can.ThreadSafeBus, can_id:int, message_timeout:float=8.0,
                 group_id:int=0x00, name="", position_threshold:float=0.001) -> None:
        """Initialize CAN communication with IDS Servo Driver.

        Args:
            can_id (int): CAN ID of target driver.
            timeout (float, optional): CAN message timeout in second.
            group_id (int, optional): Group ID of driver.
            position_threshold (float, optional): Threshold for determining if
                target position is reached.
        """
        self.can_id = can_id
        self.bus = bus
        self.message_timeout = message_timeout
        self.group_id = group_id
        self.name = name
        self.position_threshold = position_threshold

        self.is_message_received = False
        self.msg_received:can.Message = None
        self.is_running = False
        self.extension:float = read_value_from_file('/home/mon3/bky_monkey/saved_extension.txt')

        # Setup CAN listener
        can.Notifier(self.bus, [self])

    def on_message_received(self, msg:can.Message) -> None:
        """Call when CAN message is received.
        
        Implementation of can.listener.Listener callback.
        
        Args:
            msg (can.Message): Incoming CAN message
        """
        # Filter ID
        if msg.arbitration_id != self.can_id or msg.data[0] != self.group_id:
            return

        # Return message from writing to driver
        if msg.data[1] == FunctionCode.OTO_WRITE_RETURN.value:
            self.is_message_received = True
            self.msg_received = msg

        # Message from report content at interval
        elif msg.data[1] == FunctionCode.REPORT.value:
            # print("---Report content received---")
            # print_message(msg)

            # Positional report feedback
            if (msg.data[2] == RegisterAddress.READ_CURRENT_POSITION_MSB.value and
                msg.data[5] == RegisterAddress.READ_CURRENT_POSITION_LSB.value):
                extension_cnt = (msg.data[3]<<24) + (msg.data[4]<<16) +\
                                (msg.data[6]<<8) + (msg.data[7])
                self.extension = ctypes.c_int32(extension_cnt).value / self.ENCODER_CNT_PER_METRE
                # print(f"{self.name} extension = {self.extension}m")

                # return self.extension

        # Return message from reading request to driver
        elif msg.data[1] == FunctionCode.OTO_READ_RETURN.value:
            print("---Read request received---")
            print_message(msg)
            # TODO: Read request handling

    def on_error(self, exc):
        """Call when CAN message received has error.
        
        Implementation of can.listener.Listener exception.
        """

    def write_request(self, data, wait:bool=False):
        """Send write request to target driver.

        Args:
            data: CAN message data
            wait (bool, optional): Wait until message is acknowledge or timeout.
        """
        msg = can.Message(
            arbitration_id = self.can_id,
            data = data,
            is_extended_id = False
        )
        self.bus.send(msg)

        # Error handling thread
        thread = threading.Thread(target = self.write_message_thread, args=[msg])
        thread.start()

        # Wait until thread is acknowledge or timeout
        if wait:
            thread.join()

    def write_message_thread(self, msg:can.Message):
        """Thread for managing write request."""
        message_timeout = time.time() + self.message_timeout
        while True:
            # Check for receive message
            if (self.is_message_received and self.msg_received is not None
                and self.msg_received.data[2] == msg.data[2]):
                # Check if received message matches
                is_data_match = True
                for i in range(3, 8):
                    if self.msg_received.data[i] != msg.data[i]:
                        is_data_match = False
                        break

                if not is_data_match:
                    print("Warning: Message received does not match.")
                    print("---Sent---")
                    print_message(msg)
                    print("---Received---")
                    print_message(self.msg_received)
                    print()

                # Reset message
                lock = threading.Lock()
                with lock:
                    self.is_message_received = False
                    self.msg_received = None
                break

            # Check for timeout
            if time.time() > message_timeout:
                print("Warning: CAN message timeout.")
                print("---Sent---")
                print_message(msg)
                print()
                break

    def set_extension(self, ext:float, timeout:float=0, wait:bool=False):
        """Set cylinder extension target.

        Args:
            ext (float): Target extension in metre.
            timeout (float, optional): Extension wait timeout in second. 0 indicates no timeout.
            wait (bool, optional): Wait for extension is reached before running next program.
        """
        # Convert extension in metre to encoder count
        ext_cnt = int(ext * IDSServoDriver.ENCODER_CNT_PER_METRE)

        data = [
            self.group_id, # Driver's group ID
            FunctionCode.OTO_WRITE_SEND.value, # Function code
            RegisterAddress.TARGET_POSITION_MSB.value, # Register 1 (Positional control 16bit MSB)
            (ext_cnt>>24)&0xFF, # MSB
            (ext_cnt>>16)&0xFF, # LSB
            RegisterAddress.TARGET_POSITION_LSB.value, # Register 2 (Positional control 16bit LSB)
            (ext_cnt>>8)&0xFF, # MSB
            ext_cnt&0xFF, # LSB
        ]

        self.enable(True) # Enable Servo
        self.set_report_state(ReportContentType.POSITION, 100, True)

        # Create and send CAN message to driver
        self.is_running = True
        self.write_request(data)

        #Thread for checking if target position is reached
        thread = threading.Thread(target=self.extension_thread, args=[ext, timeout])
        thread.start()
        if wait:
            thread.join()

    def extension_thread(self, target:float, timeout:float):
        """Thread for managing write request.
        
        Args:
            target (float): Target extension in metre.
            timeout (float): Extension wait timeout in second. 0 indicates no timeout.
        """
        # Manage timeout value
        if timeout == 0:
            timeout = 60
        extension_timeout = time.time() + timeout

        while True:
            # Check for extension reached
            if abs(target - self.extension) < self.position_threshold:
                print("Target extension is reached, disabling servo...")
                break

            # Check for timeout
            if time.time() > extension_timeout:
                print("Warning: Extension timeout.")
                break

        self.set_report_state(ReportContentType.POSITION, 0)
        time.sleep(1)
        self.disable()
        self.is_running = False

    def enable(self, wait:bool=False):
        """Enable movement of the motor (release brake).
        
        Args:
            wait (bool, optional): Wait until message is acknowledge or timeout.
        """
        data = [
            self.group_id, # Driver's group ID
            FunctionCode.OTO_WRITE_SEND.value, # Function code
            RegisterAddress.SERVO_START_STOP.value, # Register 1 (Servo start/stop)
            0x00, # N/A
            0x01, # Start servo
            RegisterAddress.SERVO_START_STOP.value, # Register 2 (Servo start/stop)
            0x00, # N/A
            0x01, # Start servo
        ]

        # Create and send CAN message to driver
        self.write_request(data, wait)

    def disable(self, wait:bool=False):
        """Enable movement of the motor (release brake).
        
        Args:
            wait (bool, optional): Wait until message is acknowledge or timeout.
        """
        data = [
            self.group_id, # Driver's group ID
            FunctionCode.OTO_WRITE_SEND.value, # Function code
            RegisterAddress.SERVO_START_STOP.value, # Register 1 (Servo start/stop)
            0x00, # N/A
            0x00, # Stop servo
            RegisterAddress.SERVO_START_STOP.value, # Register 2 (Servo start/stop)
            0x00, # N/A
            0x00, # Stop servo
        ]

        # Create and send CAN message to driver
        self.write_request(data, wait)

    def set_report_state(self, content:ReportContentType, interval:int, wait:bool=False):
        """Set report message type and interval.

        Args:
            content (ReportContentType): Report message type enum.
            interval (int): Report interval in milliseconds.
            wait (bool, optional): Wait until message is acknowledge or timeout.
        """
        data = [
            self.group_id, # Driver's group ID
            FunctionCode.OTO_WRITE_SEND.value, # Function code
            RegisterAddress.REPORT_CONTENT.value, # Register 1 (Report message)
            0x00, # N/A
            content.value, # Content type
            RegisterAddress.REPORT_INTERVAL.value, # Register 2 (Report interval)
            (interval>>8)&0xFF, # MSB
            interval&0xFF, # LSB
        ]

        # Create and send CAN message to driver
        self.write_request(data, wait)

    # def report_position(self, content=ReportContentType.POSITION, interval:int = 100, wait:bool=False):
    #     """Set report message type and interval.

    #     Args:
    #         content (ReportContentType): Report message type enum.
    #         interval (int): Report interval in milliseconds.
    #         wait (bool, optional): Wait until message is acknowledge or timeout.
    #     """
    #     data = [
    #         self.group_id, # Driver's group ID
    #         FunctionCode.OTO_WRITE_SEND.value, # Function code
    #         RegisterAddress.REPORT_CONTENT.value, # Register 1 (Report message)
    #         0x00, # N/A
    #         content.value, # Content type
    #         RegisterAddress.REPORT_INTERVAL.value, # Register 2 (Report interval)
    #         (interval>>8)&0xFF, # MSB
    #         interval&0xFF, # LSB
    #     ]

    #     # Create and send CAN message to driver
    #     self.write_request(data, wait)

    def set_positional_control_mode(self, wait:bool=False):
        """Set control mode to positional mode."""
        data = [
            self.group_id, # Driver's group ID
            FunctionCode.OTO_WRITE_SEND.value, # Function code
            RegisterAddress.SERVO_START_STOP.value, # Register 2 (Servo start/stop)
            0x00, # N/A
            0x00, # Stop servo
            RegisterAddress.CONTROL_MODE.value, # Register 2 (Control mode selection)
            0x00, # N/A
            0xD0, # Positional control mode
        ]

        # Create and send CAN message to driver
        self.write_request(data, wait)

    def fault_reset(self, wait:bool=False):
        """Enable movement of the motor (release brake).
        
        Args:
            wait (bool, optional): Wait until message is acknowledge or timeout.
        """
        data = [
            self.group_id, # Driver's group ID
            FunctionCode.OTO_WRITE_SEND.value, # Function code
            RegisterAddress.ERROR_RESET.value, # Register 1 (Servo start/stop)
            0x00, # N/A
            0x00, # Stop servo
            0xFF,
            0x00,
            0x00,
        ]
        # Create and send CAN message to driver
        self.write_request(data, wait)

# def save_value_to_file(self, number, file_path):
#     if not isinstance(number, float):
#         raise ValueError("Input must be a float.")
#         # print("Input not integer...Saving as 0")
#     with open(file_path, 'w') as file:
#         file.write(str(number))

# def read_value_from_file(self, file_path):
#     # default_value = 0
#     try:
#         with open(file_path, 'r') as file:
#             data = file.read().strip()
#             return float(data)
    
#     except FileNotFoundError:
#         # If the file does not exist, create it and return 0
#         self.get_logger().warn(f"File {file_path} not found. Creating it with default value 0.")
#         self.save_value_to_file(0, file_path)
#         return 0
    
def main(args=None):
    """Run when this script is called."""
    # Boom 1 = 71 (0x47)
    # Boom 2 = 72 (0x48)
    # can_id_str = input("Please enter CAN ID: ")

    bus = can.ThreadSafeBus(
        interface='seeedstudio', channel='/dev/ttyUSB0', baudrate=115200, bitrate=500000
        # interface='seeedstudio', channel='/dev/ttyUSB1', baudrate=2000000, bitrate=500000
    )

    # Initialize drivers
    d1 = IDSServoDriver(bus, 1, name="Boom 1")
    time.sleep(1)
    # d2 = IDSServoDriver(bus, 72, name="Boom 2")
    d1.fault_reset()
    time.sleep(1)
    print("Set positional control mode")
    d1.set_positional_control_mode()
    # d2.set_positional_control_mode()

    time.sleep(0.5)

    print("Extend 0.1m")
    d1.set_extension(0.0, 8, False)
    while d1.is_running:
        print(f"Current Extension: {d1.extension}")

    # Max is around 1.31cm
    # MAX = 1.3 = ~95cm / Total height of monkey = 245cm
    # 1.0 = ~70+cm
    # 0.6 = ~40+cm
    # 0.8 = ~56cm
    # MIN = 0.05 = ~5cm

    # prev_ext = float(d1.extension)

    # while d1.is_running:
    #     time.sleep(0.1)
        
    #     actual_ext = float(d1.extension)
    #     print(f'Extension: {actual_ext}')

    #     if actual_ext < prev_ext:
    #         sub_pos = abs((prev_ext - actual_ext))
    #         file_pos =read_value_from_file('saved_extension.txt') - sub_pos
    #         print(f"Actual extension: {file_pos}")
    #         # feedback_msg.current_extension = file_pos
    #         save_value_to_file(file_pos, 'saved_extension.txt')
    #     #     feedback_msg.current_extension = file_pos
    #     #     # save_integer_to_file(file_pos, filepath)

    #     elif actual_ext > prev_ext:
    #         sub_pos = abs((prev_ext - actual_ext))
    #         file_pos = read_value_from_file('saved_extension.txt') + sub_pos
    #         print(f"Actual extension: {file_pos}")
    #         # feedback_msg.current_extension = file_pos
    #         save_value_to_file(file_pos, 'saved_extension.txt')
    #     #     feedback_msg.current_extension = file_pos

        
    #     # feedback_msg = SetExtension.Feedback()

    #     # Check if position has changed significantly
    #     if abs(actual_ext - prev_ext) < 0.05:
    #         break

    #     prev_ext = actual_ext



    # d2.set_extension(0.1, 8, False)
    # Wait until both actuators reached target
    # prev_ext = float(d1.extension)
    # while (d1.is_running):
    #     time.sleep(0.1)
            
    #     actual_ext = float(d1.extension)
    #     print(f"Extension: {actual_ext}")
    #     # feedback_msg.current_extension = actual_ext

    #     if actual_ext < prev_ext:
    #         sub_pos = abs((prev_ext - actual_ext))
    #         # file_pos = read_value_from_file('saved_extension.txt') - sub_pos
    #         print(f"Actual extension: {file_pos}")
    #         # save_integer_to_file(file_pos, filepath)

    #     elif actual_ext > prev_ext:
    #         sub_pos = abs((prev_ext - actual_ext))
    #         file_pos = read_value_from_file('saved_extension.txt') + sub_pos
    #         print(f"Actual extension: {file_pos}")

    # #     save_value_to_file(file_pos, 'saved_extension.txt')

    #     # Check if position has changed significantly
    #     if abs(actual_ext - prev_ext) < 0.05:
    #         break

    #     prev_ext = actual_ext
        
    #     # pass
    # print("Extension reached... Wait 3 seconds")
    # time.sleep(3)

    # # print("Extend 0m")
    # # d1.set_extension(0, 8, False)
    # # # d2.set_extension(0, 8, False)
    # # # Wait until both actuators reached target
    # # while (d1.is_running):
    # #     pass
    # # print("Extension reached... Wait 3 seconds")
    # # time.sleep(3)

def print_message(msg:can.Message):
    """Print CAN message data.

    Args:
        msg (can.Message): CAN message
    """
    print(f"id: {hex(msg.arbitration_id)}\tdata: {hex(msg.data[0])}|{hex(msg.data[1])}|\
{hex(msg.data[2])}|{hex(msg.data[3])}|{hex(msg.data[4])}|{hex(msg.data[5])}|\
{hex(msg.data[6])}|{hex(msg.data[7])}")

if __name__ == '__main__':
    main()