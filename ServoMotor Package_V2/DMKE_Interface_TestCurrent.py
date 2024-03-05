import canopen
import time
from DMKEServoDriver2 import DMKEServoDriver2_V1

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from rclpy.qos import QoSProfile



def monitor_current(instance, threshold=1, interval=0.1):
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
    prev_current = instance.read_actual_current()
    while True:
        time.sleep(interval)
        actual_current = instance.read_actual_current()

        if actual_current is not None and prev_current is not None:
            print(f"Actual current: {actual_current}")
        
            # Check if position has changed significantly
            if abs(actual_current - prev_current) < threshold:
                break
            
        prev_current = actual_current




def main(args=None):
    """Run when this script is called.
    
    1. Initialization
    2. Pre-Operational
    3. Start/Operational
    4. Stop
    
    """
    
    node_id =  0x01 # I memang dk yet

    target_speed1 = 1000
    target_speed2 = 2000
    target_speed3 = -1000
    target_speed4 = -2000
    zero_speed = 0

    network = canopen.Network()
    network.connect(interface='seeedstudio', channel='COM7', baudrate=115200, bitrate=500000)
    network.sdo_timeout = 10.0

    time.sleep(1)

    c1 = DMKEServoDriver2_V1(network, node_id)

    try:
        
        c1.NMT_Reset_Node()
        c1.NMT_Reset_Comm()

        c1.NMT_Pre_Op()
        time.sleep(2)

        c1.NMT_Start()
        time.sleep(2)

        c1.node_timeout_protection(500, 5)

        # actual_pos = c1.read_actual_pos()
        # print("Current Position = ", actual_pos)
        c1.enable()
        time.sleep(2)

        print("Setting torque control mode")
        c1.set_torque_control_mode()
        time.sleep(2)
        
        
        c1.set_current_rise(100)
        time.sleep(2)
        # c1.disable()
        # time.sleep(2)

        # c1.enable()
        c1.operating_current(300, 5)
        c1.operating_current(0, 2)
        # monitor_current(c1, threshold=1, interval=0.1)
        # c1.read_actual_current()
        
        # c1.operating_current(0)
        # time.sleep(2)
        # time.sleep(5)
        # time.sleep(1)
        c1.disable()
        time.sleep(1)

        print("Returning to PreOperational State")
        c1.NMT_Pre_Op()
        time.sleep(2)

        print("Entering Stop State")
        c1.NMT_Stop()
        time.sleep(2)

    except canopen.sdo.SdoCommunicationError:
        print("Failed to update node timeout protection")

    finally:

        print("Task completed successfully")
        network.disconnect()

if __name__ == '__main__':
    main()