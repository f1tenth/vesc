from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():

    vesc_config = os.path.join(
        get_package_share_directory('vesc'),
        'params',
        'vesc_config.yaml'
        )
    return LaunchDescription([
        Node(
            package='vesc',
            namespace='vesc',
            executable='vesc_node',
            name='vesc_node',
            parameters= [vesc_config]           
        ),

    ])

