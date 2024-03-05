import canopen
import time
from my_monkey.my_monkey.DMKEServoDriver2 import DMKEServoDriver2_V1


def monitor_position(instance, filepath, threshold=3, interval=0.05):
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
                # if count == 0:
                #     sub_pos = 
                #     real_pos = check_pos(instance, filepath) + actual_pos
                #     print(f"Actual position: {real_pos}")

                # else:
                if actual_pos < prev_pos:
                    sub_pos = abs((prev_pos - actual_pos))
                    file_pos = read_integer_from_file(filepath) - sub_pos
                    print(f"Actual position: {file_pos}")
                    # save_integer_to_file(file_pos, filepath)

                elif actual_pos > prev_pos:
                    sub_pos = abs((prev_pos - actual_pos))
                    file_pos = read_integer_from_file(filepath) + sub_pos
                    print(f"Actual position: {file_pos}")

                save_integer_to_file(file_pos, filepath)

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

# def monitor_position(instance, threshold=3, interval=0.05):
#     """
#     Monitors the position of a servo motor continuously until the change
#     in position is less than the specified threshold.

#     Args:
#     - c1: The DMKEServoDriver2_V1 object for the servo motor.
#     - threshold (optional): The threshold for considering the change in
#                             position significant. Defaults to 5.
#     - interval (optional): The time interval (in seconds) between position
#                            readings. Defaults to 0.2 seconds.
#     """
#     prev_pos = instance.read_actual_pos()
#     while True:
#         time.sleep(interval)
#         actual_pos = instance.read_actual_pos()

#         if actual_pos is not None and prev_pos is not None:
#             print(f"Actual position: {actual_pos}")
#             save_integer_to_file(actual_pos, 'dmke_encoder_pos.txt')

#             # Check if position has changed significantly
#             if abs(actual_pos - prev_pos) < threshold:
#                 read_pos = instance.read_actual_pos()
#                 print(read_pos)
#                 break
            
#         prev_pos = actual_pos

        
# def check_pos(instance, file_path):
#     data1 = instance.read_actual_pos()
#     data2 = read_integer_from_file(file_path)

#     if data1 is not None and data2 is not None:
#         if data2 - 5 <= data1 <= data2 + 5:
#             # save_integer_to_file(data1, 'dmke_encoder_pos.txt')
#             return data1

#         elif not (data2 - 5 <= data1 <= data2 + 5):
#             # save_integer_to_file(data2, 'dmke_encoder_pos.txt')
#             return data2
    
#     if data1 is None or data2 is None:
#         # save_integer_to_file(data2, 'dmke_encoder_pos.txt')
#         return data2


def save_integer_to_file(number, file_path):
    if not isinstance(number, int):
        raise ValueError("Input must be an integer.")
        # print("Input not integer...Saving as 0")

    with open(file_path, 'w') as file:
        file.write(str(number))

def read_integer_from_file(file_path):
    # default_value = 0
    # try:
    with open(file_path, 'r') as file:
        data = file.read().strip()
        return int(data)
    # except (ValueError, FileNotFoundError):
    #     # If file not found, create the file with default value
    #     with open(file_path, 'w') as file:
    #         file.write(str(default_value))
    #     return default_value
    
def real_pos(instance, target_pos, file_path):
    starting_pos = read_integer_from_file(file_path)
    read_pos = instance.read_actual_pos()
    print(f"Current Position: {starting_pos}")
    real_target = read_pos + (target_pos - starting_pos)
    return real_target


def main(args=None):
    """Run when this script is called.
    
    1. Initialization
    2. Pre-Operational
    3. Start/Operational
    4. Stop
    
    """

    node_id =  0x02 # I memang dk yet

    # count = 0

    target_position1 = 500000
    target_position2 = 200000
    zero_position = 0
    target_position3 = -500000

    network = canopen.Network()
    network.connect(interface='seeedstudio', channel='COM7', baudrate=115200, bitrate=500000)

    time.sleep(1)

    c1 = DMKEServoDriver2_V1(network, node_id)

    try:
        # c1.node_timeout_protection(250, 10)
        c1.NMT_Reset_Node()
        # c1.NMT_Reset_Comm()

        c1.NMT_Pre_Op()
        time.sleep(2)

        # c1.Node_Guard_Protocol(60)

        c1.NMT_Start()
        time.sleep(2)

        c1.enable()
        # now_pos = c1.read_actual_pos()

        # if read_integer_data() is not None:
        #     actual_pos = read_integer_data()
        # else:
        #     actual_pos = now_pos
        #     update_integer_data(actual_pos)
        # # actual_pos = read_integer_data()

        print("Setting positional control mode")
        c1.set_pos_control_mode()
        time.sleep(2)

        print("Setting Parameters for position control mode")
        c1.set_profile_velocity(2800)
        c1.set_profile_acceleration(2000)
        c1.set_profile_deceleration(2000)
        time.sleep(2)

        # Move to target position 1
        real_target1 = real_pos(c1, target_position1, 'dmke_encoder_pos.txt')

        print(f"Setting Target position to {target_position1}")
        
        c1.set_target_location(real_target1)
        time.sleep(2)

        print(f"Moving to position {target_position1}")
        # print(f"Encoder Counts Left: {real_target1}")
        c1.start_trigger_absolute()

        # real_pos = read_integer_from_file('dmke_encoder_pos.txt')
        # pospos = real_pos + c1.read_actual_pos()
        # save_integer_to_file(pospos, 'dmke_encoder_pos.txt')
        # input = check_pos(c1, 'dmke_encoder_pos.txt')
        monitor_position(c1, 'dmke_encoder_pos.txt')
        # count += 1

        time.sleep(2)
        c1.disable
        time.sleep(2)


        c1.enable()

        # Move to target position 2
        # X = check_pos(c1, 'dmke_encoder_pos.txt')
        real_target2 = real_pos(c1, target_position2, 'dmke_encoder_pos.txt')
       
        print(f"Setting Target position to {target_position2}")
        
        c1.set_target_location(real_target2)
        time.sleep(2)

        print(f"Moving to position {target_position2}")
        # print(f"Encoder Counts Left: {real_target2}")
        c1.start_trigger_absolute()

        # input = check_pos(c1, 'dmke_encoder_pos.txt')
        monitor_position(c1, 'dmke_encoder_pos.txt')

        time.sleep(2)
        c1.disable()
        time.sleep(2)


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


        c1.enable()
        # Move to zero position    
        real_target3 = real_pos(c1, zero_position, 'dmke_encoder_pos.txt')
    
        print(f"Setting Target position to {zero_position}")
        c1.set_target_location(real_target3)
        time.sleep(2)

        print(f"Moving to position {zero_position}")
        c1.start_trigger_absolute()

        # input = check_pos(c1, 'dmke_encoder_pos.txt')
        monitor_position(c1, 'dmke_encoder_pos.txt')

        time.sleep(2)
        c1.disable()
        time.sleep(2)

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