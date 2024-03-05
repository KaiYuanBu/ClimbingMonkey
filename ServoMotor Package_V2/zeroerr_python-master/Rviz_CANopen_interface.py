import canopen
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
import getch
from DMKEServoDriver2 import DMKEServoDriver2_V1

#####################################

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            qos_profile
        )

    def joint_states_callback(self, msg):
        global m1,m2, m3,m4, m5
        joint_positions = msg.position

        # Initialize CANopen network
  
        
        for i, position in enumerate(joint_positions):
            joint_name = msg.name[i]

            if joint_name == 'j1':
                 pos1 = position * 57.296
                 pos1 = pos1 * (524288 / 360)
                 m1.set_interpolation(int(pos1))
                 print('Position 1: ', pos1)
            if joint_name == 'j2':
                 pos2 = position * 57.296
                 pos2 = pos2 * (524288 / 360)
                 m2.set_interpolation(int(pos2))
                 print('Position 2: ',pos2)
            if joint_name == 'j3':
                 pos3 = position * 57.296
                 pos3 = pos3 * (524288 / 360)
                 m3.set_interpolation(int(pos3))
                 print('Position 3: ',pos3)
            if joint_name == 'j4':
                 pos4 = position * 57.296
                 pos4 = pos4 * (524288 / 360)
                 m4.set_interpolation(int(pos4))
                 print('Position 4: ',pos4)
            if joint_name == 'j5':
                pos5 = position * 57.296
                pos5 = pos5 * (524288 / 360)
                m5.set_interpolation(int(pos5))
                print('Position 5: ',pos5)                
            else: 
                print('----')

            #time.sleep(0.01)



def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()

    network = canopen.Network()
    network.connect(interface='seeedstudio', channel='/dev/ttyUSB0', baudrate=2000000, bitrate=1000000)

    time.sleep(.2)
    # speed = 1000000 
    # acceleration = 300000 
    # deceleration = 120000 

    speed = 10000
    acceleration = 10000 
    deceleration = 5000 

    global m1, m2, m3, m4, m5  

    m1 = ZeroErr(network, 41)
    m1.set_profile_velocity(speed)
    m1.set_profile_deceleration(deceleration)
    m1.set_profile_acceleration(acceleration)
    m1.enable()

    time.sleep(.2)

    m2 = ZeroErr(network, 42)
    m2.set_profile_velocity(speed)
    m2.set_profile_deceleration(deceleration)
    m2.set_profile_acceleration(acceleration)
    m2.enable()

    time.sleep(.2)

    m3 = ZeroErr(network, 43)
    m3.set_profile_velocity(speed)
    m3.set_profile_deceleration(deceleration)
    m3.set_profile_acceleration(acceleration)
    m3.enable()

    time.sleep(.2)

    m4 = ZeroErr(network, 44)
    m4.set_profile_velocity(speed*1.8)
    m4.set_profile_deceleration(deceleration)
    m4.set_profile_acceleration(acceleration)
    m4.enable()

    time.sleep(.2)
    
    m5 = ZeroErr(network, 45)
    m5.set_profile_velocity(speed*1.8)
    m5.set_profile_deceleration(deceleration)
    m5.set_profile_acceleration(acceleration)
    m5.enable() 

    time.sleep(.2)  
    
    
    try:
        while True:
            # Perform other operations or tasks within the loop
            rclpy.spin(joint_state_subscriber)
            print("Loop running...")
            time.sleep(.2) 
    except KeyboardInterrupt:
        
        m1.set_interpolation(1000) #Send motortoinitial position
        m2.set_interpolation(1000)
        m3.set_interpolation(1000)
        m4.set_interpolation(1000)      
        m5.set_interpolation(1000)
        time.sleep(10)             #Delay for motor to move to initialposition
        print('Operation ending...')
        m1.disable()          
        m2.disable() 
        m3.disable() 
        m4.disable()
        m5.disable()
    
        
    network.disconnect()

if __name__ == '__main__':
    main()