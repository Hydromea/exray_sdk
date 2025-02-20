from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='examples',
            executable='rotating_square_motion_demo',
            name='rotating_square_motion_demo_node',
            output='screen',
        ),
    ])
