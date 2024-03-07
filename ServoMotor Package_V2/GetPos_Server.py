from dmke_interface.srv import GetPosition     # CHANGE

import rclpy
from rclpy.node import Node
import canopen
import time
from my_monkey.my_monkey.DMKEServoDriver2 import DMKEServoDriver2_V1


class GetPosService(Node):

    def __init__(self):
        super().__init__('get_pos_service')
        self.srv = self.create_service(GetPosition, 'get_position', self.get_position_callback)        # CHANGE

        node_id =  0x01
        self.network = canopen.Network()
        self.network.connect(interface='seeedstudio', 
                             channel='/dev/ttyS0', 
                             baudrate=115200, 
                             bitrate=500000)

        time.sleep(1)
        self.c1 = DMKEServoDriver2_V1(self.network, node_id)
        self.c1.NMT_Reset_Node()
        self.c1.NMT_Reset_Comm()
        
        self.c1.NMT_Pre_Op()
        time.sleep(2)
        
        self.c1.NMT_Start()
        time.sleep(2)


    def get_position_callback(self, response):
        response.position_gotten = self.c1.read_actual_pos()                                             # CHANGE
        self.get_logger().info('Position Obtained: %u ' % (response.position_gotten)) # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    get_pos_service = GetPosService()

    rclpy.spin(get_pos_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
