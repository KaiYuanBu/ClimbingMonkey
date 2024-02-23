"""
Driver utility for ZeroErr servos. Example of use below.

Written by:
1. Kean Hao

Last updated: 29/11/2023
"""

import canopen
import ctypes

class ZeroErr:
    """
    Driver class for ZeroErr servo motors.
    """

    def __init__(self, network:canopen.Network, node_id) -> None:
        """Initialize ZeroErr servo node in CANopen network.

        Args:
            network (canopen.Network): Target CANopen network.
            node_id (int): CANopen node ID of the servo.
        """
        self.node = network.add_node(node_id, f'DCHCAN.eds')
        # self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.nmt.state = 'OPERATIONAL'

        # Set operation mode
        try:
            print("Setting Opreation Mode to Interpolation position mode")
            operation_mode = self.node.sdo[0x6060] # Modes of operation
            operation_mode.raw = 0x01 # position control mode

            # operation_mode_recv = m1_node.sdo[0x6061]
            # print(operation_mode_recv.get_data)
        except canopen.sdo.SdoCommunicationError:
            # TODO: Control mode not set
            print("Failed to set Operation Mode to Interpolation position mode")

    def set_profile_velocity(self, velocity):
        """Set profile velocity to canopen node (CiA 402).

        Args:
            velocity (int): Profile velocity in cnt/s.
        """
        try:
            profile_velocity = self.node.sdo[0x6081] # Profile velocity
            profile_velocity.raw = velocity
        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to set Profile velocity to {velocity}cnt/s")

    def set_profile_acceleration(self, acceleration):
        """Set profile acceleration to canopen node (CiA 402).

        Args:
            acceleration (int): Profile acceleration in cnt/s^2.
        """
        try:
            profile_acceleration = self.node.sdo[0x6083] # Profile acceleration
            profile_acceleration.raw = acceleration
        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to set Profile acceleration to {acceleration}cnt/s^2")

    def set_profile_deceleration(self, deceleration):
        """Set profile deceleration to canopen node (CiA 402).

        Args:
            deceleration (int): Profile deceleration in cnt/s^2.
        """
        try:
            profile_deceleration = self.node.sdo[0x6084] # Profile deceleration
            profile_deceleration.raw = deceleration
        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to set Profile deceleration to {deceleration}cnt/s^2")

    def set_interpolation(self, position : int):
        """Set interpolation to canopen node (CiA 402).

        Args:
            position (int): Target position in interpolation (cnt).
        """
        try:
            # Interpolation data record, Parameter1 of ip function
            interpolation1 = self.node.sdo[0x60C1][0x01]
            # Interpolation data record, Parameter2 of ip function
            interpolation2 = self.node.sdo[0x60C1][0x02]
            interpolation1.raw = ctypes.c_int16(position & 0xFFFF).value # Interpolation least signification 2 byte
            interpolation2.raw = ctypes.c_int16((position >> 16) & 0xFFFF).value # Interpolation most signification 2 byte
        except canopen.sdo.SdoCommunicationError:
            print(f"Failed to set interpolation position to {position}cnt/s^2")

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
            operation_mode.raw = 0x06 # Disable
        except canopen.sdo.SdoCommunicationError:
            # TODO: Control mode not set
            print("Failed to set controlword to disable")



