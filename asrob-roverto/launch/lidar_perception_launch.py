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
                    FindPackageShare('ydlidar_ros2_driver'),
                    'launch',
                    'ydlidar_launch.py'
                ])
            ]),
        ),
        Node(
            package='asrob-roverto',
            namespace='asrob-roverto',
            executable='lidar',
            name='lidar_detection'
        )
    ])
