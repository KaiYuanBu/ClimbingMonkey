import canopen
import time
from DMKEServoDriver2 import DMKEServoDriver2_V1

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from rclpy.qos import QoSProfile



def monitor_position(instance, threshold=5, interval=0.2):
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
        
            # Check if position has changed significantly
            if abs(actual_pos - prev_pos) < threshold:
                break
            
        prev_pos = actual_pos




def main(args=None):
    """Run when this script is called.
    
    1. Initialization
    2. Pre-Operational
    3. Start/Operational
    4. Stop
    
    """

    node_id =  0x01 # I memang dk yet

    target_position1 = 200000
    target_position2 = 500000
    zero_position = 0
    target_position3 = -500000

    network = canopen.Network()
    network.connect(interface='seeedstudio', channel='COM7', baudrate=115200, bitrate=500000)

    time.sleep(1)

    c1 = DMKEServoDriver2_V1(network, node_id)

    try:
        # c1.node_timeout_protection(250, 10)

        c1.NMT_Pre_Op()
        time.sleep(2)

        c1.Node_Guard_Protocol(60)

        # c1.NMT_Start()
        # time.sleep(2)

        # c1.enable()
        # actual_pos = c1.read_actual_pos()
        # print("Current Position = ", actual_pos)

        # print("Setting positional control mode")
        # c1.set_pos_control_mode()
        # time.sleep(2)

        # print("Setting Parameters for position control mode")
        # c1.set_profile_velocity(2000)
        # c1.set_profile_acceleration(100)
        # c1.set_profile_deceleration(100)
        # time.sleep(2)

        # # Move to target position 1
        # print(f"Setting Target position to {target_position1}")
        # c1.set_target_location(target_position1)
        # time.sleep(2)

        # print(f"Moving to position {target_position1}")
        # c1.start_trigger_absolute()

        # monitor_position(c1, threshold=5, interval=0.2)

        # time.sleep(2)
        # c1.disable
        # time.sleep(2)


        # c1.enable()

        # # Move to target position 2
        # print(f"Setting Target position to {target_position2}")
        # c1.set_target_location(target_position2)
        # time.sleep(2)

        # print(f"Moving to position {target_position2}")
        # c1.start_trigger_absolute()

        # monitor_position(c1, threshold=5, interval=0.2)

        # time.sleep(2)
        # c1.disable()
        # time.sleep(2)


        # c1.enable()
        # # Move to zero position    
        # print(f"Setting Target position to {target_position3}")
        # c1.set_target_location(target_position3)
        # time.sleep(2)

        # print(f"Moving to position {target_position3}")
        # c1.start_trigger_absolute()

        # monitor_position(c1, threshold=5, interval=0.2)


        # time.sleep(2)
        # c1.disable()
        # time.sleep(2)


        # c1.enable()
        # # Move to zero position    
        # print(f"Setting Target position to {zero_position}")
        # c1.set_target_location(zero_position)
        # time.sleep(2)

        # print(f"Moving to position {zero_position}")
        # c1.start_trigger_absolute()

        # monitor_position(c1, threshold=5, interval=0.2)


        # time.sleep(2)
        # c1.disable()
        # time.sleep(2)



        # print("Returning to PreOperational State")
        # c1.NMT_Pre_Op()
        # time.sleep(2)

        # print("Entering Stop State")
        # c1.NMT_Stop()
        # time.sleep(2)

    except canopen.sdo.SdoCommunicationError:
        print("Failed to update node timeout protection")

    finally:

        print("Task completed successfully")
        network.disconnect()

if __name__ == '__main__':
    main()