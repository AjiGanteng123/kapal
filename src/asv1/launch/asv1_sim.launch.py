from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('asv1')
    world_path = os.path.join(share, 'config', 'asv1_world.sdf')
    config_path = os.path.join(share, 'config', 'params_sim.yaml')

    bridges = [
        '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        '/model/kapal/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
        '/model/kapal/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/model/kapal/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
    ]

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '4', world_path],
            output='screen',

        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=bridges,
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='asv1',
            executable='sim_bridge.py',
            name='sim_bridge',
            parameters=[config_path],
            output='screen',
        ),

        Node(
            package='asv1',
            executable='node_deteksi.py',
            name='node_deteksi',
            parameters=[config_path],
            output='screen',
        ),

        Node(
            package='asv1',
            executable='node_navigasi',
            name='node_navigasi',
            parameters=[config_path],
            output='screen',
        ),
    ])
