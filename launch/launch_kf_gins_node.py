from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kf_gins',
            namespace='kf_gins',
            executable='kf_gins_node',
            name='kf_gins_1'
        )
    ])
