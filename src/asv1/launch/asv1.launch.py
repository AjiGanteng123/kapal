from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('asv1'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='asv1',
            executable='node_kamera.py',
            name='node_kamera',
            parameters=[config_path],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='asv1',
            executable='node_deteksi.py',
            name='node_deteksi',
            parameters=[config_path],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='asv1',
            executable='node_lidar.py',
            name='node_lidar',
            parameters=[config_path],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='asv1',
            executable='node_navigasi',
            name='node_navigasi',
            parameters=[config_path],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='asv1',
            executable='node_motor.py',
            name='node_motor',
            parameters=[config_path],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='asv1',
            executable='node_misi.py',
            name='node_misi',
            parameters=[config_path],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='asv1',
            executable='viewer.py',
            name='viewer',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
