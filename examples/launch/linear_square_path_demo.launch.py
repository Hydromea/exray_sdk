from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='examples',
            executable='linear_square_path_demo',
            name='linear_square_path_demo_node',
            output='screen',
        ),
    ])
