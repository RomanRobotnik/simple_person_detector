from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('simple_person_detector'),
        'config',
        'simple_person_detector.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='YAML file with detector parameters',
    )

    node = Node(
        package='simple_person_detector',
        executable='detector_node',
        name='person_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
        ],
    )

    return LaunchDescription([
        config_arg,
        node,
    ])