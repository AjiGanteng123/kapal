from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('robot_stack'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_stack',
            executable='lidar_node',
            name='lidar_node',
            parameters=[config_path],
            output='screen'
        ),
        Node(
            package='robot_stack',
            executable='vision_node.py',
            name='vision_node',
            parameters=[config_path],
            output='screen'
        ),
        Node(
            package='robot_stack',
            executable='autonomous_node.py',
            name='autonomous_node',
            parameters=[config_path],
            output='screen'
        ),
        Node(
            package='robot_stack',
            executable='motor_node',
            name='motor_node',
            parameters=[config_path],
            output='screen'
        ),
    ])
