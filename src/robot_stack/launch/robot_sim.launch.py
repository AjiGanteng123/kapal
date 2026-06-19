from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('robot_stack'),
        'config', 'robot_world.sdf'
    )

    config_path = os.path.join(
        get_package_share_directory('robot_stack'),
        'config', 'params.yaml'
    )

    return LaunchDescription([
        # Gazebo simulator with the world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '4', world_path],
            output='screen',
        ),

        # Bridge: Gazebo LiDAR topic → ROS2 /scan
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_scan',
            arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            output='screen',
        ),

        # Bridge: Gazebo camera topic → ROS2 /camera/image_raw
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_camera',
            arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen',
        ),

        # robot_stack nodes
        Node(
            package='robot_stack',
            executable='lidar_node',
            name='lidar_node',
            parameters=[config_path],
            output='screen',
        ),
        Node(
            package='robot_stack',
            executable='motor_node',
            name='motor_node',
            parameters=[config_path],
            output='screen',
        ),
        Node(
            package='robot_stack',
            executable='vision_node.py',
            name='vision_node',
            parameters=[config_path],
            output='screen',
        ),
        Node(
            package='robot_stack',
            executable='autonomous_node.py',
            name='autonomous_node',
            parameters=[config_path],
            output='screen',
        ),
    ])
