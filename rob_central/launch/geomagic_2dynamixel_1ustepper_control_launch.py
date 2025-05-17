from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamixel_io',
            executable='read_write_node',
            name='dynamixel_io_node',
            output='log',
            # parameters=['config/params.yaml'],
            arguments=['--ros-args', '--log-level', 'warn'],
            # remappings=[('/old/topic', '/new/topic')],
        ),
        Node(
            package='omni_common',
            executable='omni_state',
            name='haptic_dev_node',
            output='screen',
            # emulate_tty=True,
            # parameters=['config/params.yaml'],
        ),
        Node(
            package='rob_central',
            executable='central_node',
            name='central_node',
            output='screen',
            # emulate_tty=True,
            # parameters=['config/params.yaml'],
        ),
        Node(
            package='ustepper_dev', 
            executable='ustepper_node',
            name='ustepper_node',
            output='screen',
            # emulate_tty=True,
            # parameters=['config/params.yaml'],
        ),
    ])
