import canopen
import time
from ZeroErr import ZeroErr

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from rclpy.qos import QoSProfile


def main(args=None):
    # Initialize CANopen network
    network = canopen.Network()
    # network.connect(channel='COM7', bustype ='seeedstudio', bitrate=500000)

    # network.connect('seeedstudio', 'COM7')
    # can.interface.Bus(channel=channel, bitrate=bitrate)
    # network.connect(bustype='kvaser', channel=0, bitrate=250000)
    # network.connect(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
    # network.connect(bustype='ixxat', channel=0, bitrate=250000)
    # network.connect(bustype='nican', channel='CAN0', bitrate=250000)
    network.connect(interface='seeedstudio', channel='COM7', baudrate=2000000, bitrate=500000)

    time.sleep(1)

    m1 = ZeroErr(network, 1)



    m1.set_profile_velocity(100000)
    m1.set_profile_deceleration(50000)
    m1.set_profile_acceleration(50000)

    # m2 = ZeroErr(network, 46)
    # m2.set_profile_velocity(10000)
    # m2.set_profile_deceleration(5000)
    # m2.set_profile_acceleration(5000)
    # m2.enable()

    # m3 = ZeroErr(network, 47)
    # m3.set_profile_velocity(10000)
    # m3.set_profile_deceleration(5000)
    # m3.set_profile_acceleration(5000)
    # m3.enable()

    # time.sleep(2)
    # m1.enable()
    # time.sleep(0.5)
    # m1.set_interpolation(0)
    # print(f"Set interpolation 0")
    # time.sleep(5)

    # pos = 180 * (524288 / 360)
    # m1.set_interpolation(int(pos))
    # print(f"Joint position: {pos}")
    # m2.set_interpolation(10000)
    # m3.set_interpolation(10000)

    #time.sleep(10)

    #m1.set_interpolation(-131072)
    # m2.set_interpolation(-10000)
    # m3.set_interpolation(-10000)

    # time.sleep(5)
    
    m1.disable()

    network.disconnect()

if __name__ == '__main__':
    main()
