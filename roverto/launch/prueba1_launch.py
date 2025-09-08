from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('orbbec_camera'),
                    'launch',
                    'astra_pro_plus.launch.py'
                ])
            ]),
        ),
        Node(
            package='roverto',
            namespace='roverto',
            executable='number',
            name='number_detection'
        ),
        Node(
            package='roverto',
            namespace='roverto',
            executable='box_color',
            name='box_color_detection'
        ),
        Node(
            package='roverto',
            namespace='roverto',
            executable='prueba1',
            name='perception_spin',
            output='screen'
        )
    ])
