import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler, TimerAction

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('my_monkey'), 'config', 'dmke_hardware.yaml'
    )

    node1 = Node(
        package='my_monkey',
        executable='DMKE_Interface_ROS',
        name='c1',
        namespace='/arm',
        parameters=[config]
    )

    # mock_b1_node = Node(
    #     package='arm_bt',
    #     executable='mock_actuator',
    #     name='b1',
    #     namespace='/arm',
    #     parameters=[config]
    # )

    # mock_b2_node = Node(
    #     package='arm_bt',
    #     executable='mock_actuator',
    #     name='b2',
    #     namespace='/arm',
    #     parameters=[config]
    # )

    return LaunchDescription([
        node1,
    ])