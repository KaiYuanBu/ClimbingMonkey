import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import canopen
import time

from DMKEServoDriver.DMKEServoDriver import DMKEServoDriver2_V1


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.publisher = self.create_publisher(Int32, '/motor_actual_position', 10)
        self.msg = Int32()
        self.subscription = self.create_subscription(
            Int32,
            '/motor_actual_position',
            self.monitor_position,
            10)
        self.subscription  # prevent unused variable warning
        # self.target_positions = [200000, 500000, -500000, 0]  # List of target positions
        # self.zero_position = 0
        # self.current_target_index = 0
        # self.position_threshold = 5
        # self.position_interval = 0.2

        node_id = 0x01  # Your motor node ID

        self.network = canopen.Network()
        self.network.connect(interface='seeedstudio', channel='/dev/ttyS0', baudrate=115200, bitrate=500000)
        time.sleep(1)

        self.c1 = DMKEServoDriver2_V1(self.network, node_id)

        self.c1.NMT_Reset_Node()
        self.c1.NMT_Reset_Comm()
        self.c1.NMT_Pre_Op()
        time.sleep(2)
        self.c1.NMT_Start()
        time.sleep(2)

        self.c1.enable()

        self.c1.set_pos_control_mode()
        time.sleep(2)
        self.c1.set_profile_velocity(2500)
        self.c1.set_profile_acceleration(1000)
        self.c1.set_profile_deceleration(1000)
        time.sleep(2)


    # def position_callback(self, msg):
    #     actual_pos = msg.data
    #     if self.c1 is not None:
    #         # Check if position has changed significantly
    #         if abs(actual_pos - self.target_positions[self.current_target_index]) < self.position_threshold:
    #             self.current_target_index = (self.current_target_index + 1) % len(self.target_positions)
    #             new_target_pos = self.target_positions[self.current_target_index]
    #             self.set_target_position(new_target_pos)

    # def set_target_position(self, target_pos):
    #     if self.c1 is not None:
    #         self.c1.set_target_location(target_pos)
    #         self.get_logger().info(f"Moving to position: {target_pos}")

    def monitor_position(self, instance, threshold=5, interval=0.2):
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
                # Publish actual position
                self.msg.data = actual_pos
                self.publisher.publish(self.msg)

                # Check if position has changed significantly
                if abs(actual_pos - prev_pos) < threshold:
                    break
                
            prev_pos = actual_pos


    def run_motor(self):
        
        target_position1 = 200000
        target_position2 = 400000
        target_position3 = 600000
        target_position4 = 0


        try:
            
             # Move to target position 1
            print(f"Setting Target position to {target_position1}")
            self.c1.set_target_location(target_position1)
            time.sleep(2)

            print(f"Moving to position {target_position1}")
            self.c1.start_trigger_absolute()

            self.monitor_position(self.c1, threshold=5, interval=0.2)

            time.sleep(2)
            self.c1.disable
            time.sleep(2)


            self.c1.enable()

            # Move to target position 2
            print(f"Setting Target position to {target_position2}")
            self.c1.set_target_location(target_position2)
            time.sleep(2)

            print(f"Moving to position {target_position2}")
            self.c1.start_trigger_absolute()

            self.monitor_position(self.c1, threshold=5, interval=0.2)

            time.sleep(2)
            self.c1.disable()
            time.sleep(2)


            self.c1.enable()
            # Move to zero position    
            print(f"Setting Target position to {target_position3}")
            self.c1.set_target_location(target_position3)
            time.sleep(2)

            print(f"Moving to position {target_position3}")
            self.c1.start_trigger_absolute()

            self.monitor_position(self.c1, threshold=5, interval=0.2)


            time.sleep(2)
            self.c1.disable()
            time.sleep(2)


            self.c1.enable()
            # Move to zero position    
            print(f"Setting Target position to {target_position4}")
            self.c1.set_target_location(target_position4)
            time.sleep(2)

            print(f"Moving to position {target_position4}")
            self.c1.start_trigger_absolute()

            self.monitor_position(self.c1, threshold=5, interval=0.2)


            time.sleep(2)
            self.c1.disable()
            time.sleep(2)



            print("Returning to PreOperational State")
            self.c1.NMT_Pre_Op()
            time.sleep(2)

            print("Entering Stop State")
            self.c1.NMT_Stop()
            time.sleep(2)

            self.get_logger().info("Task completed successfully")
            self.network.disconnect()

        except canopen.sdo.SdoCommunicationError:
            self.get_logger().error("Failed to update node timeout protection")



def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorControlNode()
    motor_node.run_motor()
    rclpy.spin_once(motor_node)
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
