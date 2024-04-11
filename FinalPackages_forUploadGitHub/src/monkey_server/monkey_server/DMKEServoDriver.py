"""
Python driver class for DMKE Servo Driver.

Written by:
    1. 
"""

# from enum import Enum

# import threading
import time
# import ctypes
import canopen

# FUNCTION CODE + NODE ID + COMMAND + INDEX LOW BYTE + INDEX HIGH BYTE + SUBINDEX + D1 + D2 + D3 + D4
# \                     /
#  \___________________/
#      Arbitration ID       DATA[0] +    DATA[1]     +     DATA[2]     +  DATA[3] + DATA[4] ~ DATA[7]


class DMKEServoDriver:
    """Driver class for ZeroErr servo motors"""

    def __init__(self, network:canopen.Network, node_id) -> None:
        """Initialize DMKE servo node in CANopen network.

        Args:
            network (canopen.Network): Target CANopen network.
            node_id (int): CANopen node ID of the servo.
        """
        # self.node = network.add_node(node_id, f'DCHCAN.eds')

        # self.node.nmt.state = 'PRE-OPERATIONAL'

        # Set operation mode
        try:
            self.node = network.add_node(node_id, '/home/mon3/FinalPackages2/src/monkey_server/monkey_server/DCHCAN.eds')
            print("Node Initialized:)")
            # self.NMT_Start()

        except canopen.sdo.SdoCommunicationError:
            # TODO: Control mode not set
            print("Failed to set NMT Operation Mode")


    def enable(self):
        """Set controlword enable."""
        try:
            print("Setting controlword to enable")
            operation_mode = self.node.sdo[0x6040] # Controlword
            operation_mode.raw = 0x0F # enable
        except canopen.sdo.SdoCommunicationError:
            # TODO: Control mode not set
            print("Failed to set controlword to enable")


    def disable(self):
        """Set controlword disable."""
        try:
            print("Setting controlword to disable")
            operation_mode = self.node.sdo[0x6040] # Controlword
            operation_mode.raw = 0x00 # Disable
        except canopen.sdo.SdoCommunicationError:
            # TODO: Control mode not set
            print("Failed to set controlword to disable")





    def NMT_Start(self):
        """Send a Network Management (NMT) START command to a node in a CANopen network.
        """
        self.node.nmt.send_command(0x01)
        print("NMT Operation Mode Started")
        # self.write_msg(data, wait)

    def NMT_Stop(self):
        """Send a Network Management (NMT) STOP command to a node in a CANopen network.
        """
        self.node.nmt.send_command(0x02)
        print("NMT Operation Mode Stopped")

    def NMT_Pre_Op(self):
        """Send a Network Management (NMT) PRE-OPERATIONAL command to a node in a CANopen network.
        """
        self.node.nmt.send_command(0x80)
        print("NMT Pre Operation Mode Activated")

    def NMT_Reset_Node(self):
        """Send a Network Management (NMT) RESET NODE command to a node in a CANopen network.
        """
        self.node.nmt.send_command(0x81)

    def NMT_Reset_Comm(self):
        """Send a Network Management (NMT) RESET COMMUNICATION command to a node in a CANopen network.
        """
        self.node.nmt.send_command(0x82)
        print("RESET COMM REQUESTED")


  ################################################################################################
    def Node_Guard_Protocol(self, period):
        """Send a Node Guarding message to monitor the operational state of a node in a CANopen network.
        """
        
        self.node.nmt.start_node_guarding(period)
        self.node.nmt.stop_node_guarding()

  ################################################################################################3

    def set_pos_control_mode(self):
        """Set position control mode for canopen node (CiA 402).

        """
        try:
            
            control_word_value = 0x01  # ControlWord value for position control mode
            control_word_bytes = control_word_value.to_bytes(1, byteorder='little')  # Convert to bytes
            pos_control_mode = self.node.sdo[0x6060]
            pos_control_mode.raw = control_word_bytes

        except canopen.sdo.SdoCommunicationError:
            # TODO: Control mode not set
            print("Set Position Control Mode Failure")


    def set_profile_velocity(self, velocity):
        """Set profile velocity to canopen node (CiA 402).

        Args:
            velocity (int): Profile velocity in rpm.
        """
        try:
            profile_velocity = self.node.sdo[0x6081] # Profile velocity

            velocityX = int(((velocity/0.1)/60)*10000)
            velocity_bytes = velocityX.to_bytes(4, byteorder='little')
            profile_velocity.raw = velocity_bytes
        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to set Profile velocity to {velocity}rpm")


    def set_profile_acceleration(self, acceleration):
        """Set profile acceleration to canopen node (CiA 402).

        Args:
            acceleration (int): Profile acceleration in r/s^2.
        """
        try:
            profile_acceleration = self.node.sdo[0x6083] # Profile acceleration
            accel_bytes = acceleration.to_bytes(4, byteorder='little')
            profile_acceleration.raw = accel_bytes
        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to set Profile acceleration to {acceleration}r/s^2")

    def set_profile_deceleration(self, deceleration):
        """Set profile deceleration to canopen node (CiA 402).

        Args:
            deceleration (int): Profile deceleration in r/s^2.
        """
        try:
            profile_deceleration = self.node.sdo[0x6084] # Profile deceleration
            decel_bytes = deceleration.to_bytes(4, byteorder='little')
            profile_deceleration.raw = decel_bytes
        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to set Profile deceleration to {deceleration}r/s^2")

    def set_target_location(self, position : int):
        """Set target position to canopen node (CiA 402).

        Args:
            position (int): Target position in interpolation (cnt).
        """
        try:
            target_pos = self.node.sdo[0x607A]

            if position >= 0:
                pos_bytes = position.to_bytes(4, byteorder='little')

            elif position < 0:
                pos_bytes = position.to_bytes(4, byteorder='little', signed=True)

            target_pos.raw = pos_bytes

        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to set target position to {position}cnt")


    def start_trigger_absolute(self):
        try:
            start_trig_abs = self.node.sdo[0x6040] # Control Word
            trig_data1 = 0x1F
            start_trig_bytes = trig_data1.to_bytes(2, byteorder='little')
            start_trig_abs.raw = start_trig_bytes
        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to start trigger for absolute position")

    def start_trigger_relative(self):
        try:
            start_trig_rel = self.node.sdo[0x6040] # Control Word
            trig_data2 = 0x5F
            start_trig_bytes = trig_data2.to_bytes(2, byteorder='little')
            start_trig_rel.raw = start_trig_bytes
        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to start trigger for absolute position")


    # def read_actual_pos(self):
    #     try:
    #         actual_pos = self.node.sdo[0x6064] # Control Word
    #         # current_pos.raw = 0x00

    #         actual_pos.read()  # Read the actual speed
    #         actual_pos = actual_pos.raw  # Get the raw bytes

    #         # Interpret the bytes based on the data format
    #         # Here, we assume the actual speed is a 32-bit signed integer (4 bytes)
    #         # position = int.from_bytes(actual_pos_bytes, byteorder='little', signed=True)
        
    #         return actual_pos  # Return the actual speed value
        
    #     except canopen.sdo.SdoCommunicationError:
    #         print(f"Failed to read current position of motor")
    #         self.read_actual_pos()
            
    def read_actual_pos(self, max_attempts=5):
        attempts = 0
        while attempts < max_attempts:
            try:
                actual_pos = self.node.sdo[0x6064] # Control Word
                actual_pos.read()  # Read the actual position
                actual_pos = actual_pos.raw  # Get the raw bytes
    
                return actual_pos  # Return the actual position value
            
            except canopen.sdo.SdoCommunicationError:
                print(f"Failed to read current position of motor. Retrying...")
                attempts += 1
    
        print(f"Failed to read current position after {max_attempts} attempts.")
        return None  # Or whatever you want to return in case of failure


    def set_speed_control_mode(self):
        """Set speed control mode for canopen node (CiA 402).

        """
        try:
            speed_control_mode = self.node.sdo[0x6060]

            data_value = 0x03  # ControlWord value for position control mode
            data_bytes = data_value.to_bytes(1, byteorder='little')  # Convert to bytes
            speed_control_mode.raw = data_bytes

        except canopen.sdo.SdoCommunicationError:
            # TODO: Control mode not set
            print("Set Speed Control Mode Failure")

    def write_speed(self, speed:int):
        """Set target speed for speed control mode(CiA 402).

        """
        try:
            write_speed = self.node.sdo[0x60FF]

            if speed >= 0:
                speed_data = int(((speed/0.1)/60)*10000)
                data_bytes = speed_data.to_bytes(4, byteorder='little')  # Convert to bytes

            elif speed < 0:
                speed_data = int((((speed)/0.1)/60)*10000)
                data_bytes = speed_data.to_bytes(4, byteorder='little', signed=True)

            write_speed.raw = data_bytes

        except canopen.sdo.SdoCommunicationError:
            # TODO: Control mode not set
            print("Speed write request in speed control mode FAILURE")


    def read_actual_speed(self):
        try:
            actual_speed = self.node.sdo[0x6069] # Control Word
            # actual_speed.raw = 0x00
            actual_speed.read()  # Read the actual speed
            actual_speed_bytes = actual_speed.raw  # Get the raw bytes

            # Interpret the bytes based on the data format
            # Here, we assume the actual speed is a 32-bit signed integer (4 bytes)
            speed = int.from_bytes(actual_speed_bytes, byteorder='little', signed=True)
        
            return speed  # Return the actual speed value

        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to read current speed of motor")




    def set_torque_control_mode(self):
        """Set torque control mode for canopen node (CiA 402).

        """
        try:
            torque_control_mode = self.node.sdo[0x6060]

            data_value = 0x04
            data_bytes = data_value.to_bytes(1, byteorder='little')  # Convert to bytes
            torque_control_mode.raw = data_bytes
            print("Setting Torque/Current Control Mode")


        except canopen.sdo.SdoCommunicationError:
            # TODO: Control mode not set
            print("Set Torque Control Mode Failure")


    def set_current_rise(self, crr:int):
        """Set current rise rate of motor

            Args:
                crr = current rise rate (mA/s)
        """
        try:
            current_rise_rate = self.node.sdo[0x2113]

            data_bytes = crr.to_bytes(4, byteorder='little')  # Convert to bytes
            current_rise_rate.raw = data_bytes
            print(f"Setting Current Rise to {crr}")


        except canopen.sdo.SdoCommunicationError:
            # TODO: Control mode not set
            print("Set Torque Control Mode Failure")


    def operating_current(self, op_current:int, duration:int):
        """Set current rise rate of motor

            Args:
                op_current = operating current (mA)
        """
        try:
            current_rise_rate = self.node.sdo[0x2340]

            data_bytes = op_current.to_bytes(2, byteorder='little')  # Convert to bytes
            current_rise_rate.raw = data_bytes

            # Start time
            start_time = time.time()

            # Run the operation for the specified duration
            while time.time() - start_time < duration:
                # Add any other operations you want to perform here
                pass

            # Once the duration is reached, disable the operation
            current_rise_rate.raw = b'\x00\x00'  # Set current to 0


        except canopen.sdo.SdoCommunicationError:
            # TODO: Control mode not set
            print("Set Torque Control Mode Failure")


    def read_actual_current(self, max_attempts = 5):
        attempts = 0
        while attempts < max_attempts:
            try:
                actual_current = self.node.sdo[0x221C] # Control Word
                # current.raw = 0x00

                actual_current.read()  # Read the actual speed
                actual_current_bytes = actual_current.raw  # Get the raw bytes

                # Interpret the bytes based on the data format
                # Here, we assume the actual speed is a 32-bit signed integer (4 bytes)
                # current = int.from_bytes(actual_current_bytes, byteorder='little', signed=True)

                return actual_current_bytes  # Return the actual speed value
        
            except canopen.sdo.SdoCommunicationError:
                print(f"Failed to read actual current of motor, Retrying...")
                attempts += 1
            
            print(f"Failed to read current position after {max_attempts} attempts.")
            return None  # Or whatever you want to return in case of failure
                
    
        



    def node_timeout_protection(self, timeout:int, factor:int):
        '''
            Args: 
            timeout = node protection time in ms
            factor = node protection factor

            EX: the total node timeout is 250 * 4 = 1000ms. If the remote frame is not received within
                1s, the node stops
        '''
        try:
            timeout_protection = self.node.sdo[0x100C] # Control Word

            data_bytes = timeout.to_bytes(2, byteorder='little') # Timeout in ms
            timeout_protection.raw = data_bytes

            # time.sleep(0.2)

            timeout_factor = self.node.sdo[0x100D]

            data_bytes2 = factor.to_bytes(1, byteorder='little') # factor
            timeout_factor.raw = data_bytes2
        
        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to update node timeout protection")
        
    
    # For Syncing:
    # # Transmit every 10 ms
    # network.sync.start(0.01)

    # network.sync.stop()
