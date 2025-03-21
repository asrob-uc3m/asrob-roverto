from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            namespace='perception1',
            executable='number',
            name='number_detection'
        ),
        Node(
            package='perception',
            namespace='perception2',
            executable='box_color',
            name='box_color_detection'
        ),
        Node(
            package='perception',
            namespace='perception3',
            executable='spin',
            name='perception_spin'
        )
    ])