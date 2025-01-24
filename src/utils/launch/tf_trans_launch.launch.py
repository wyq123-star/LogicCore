import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

def generate_launch_description():
    # Get the launch file directory
    utils_dir = get_package_share_directory('utils')
    # Create the launch configuration variables
    # Declare the launch options
    return LaunchDescription([
        Node(
            package='utils',
            executable='tf_trans',
            output='screen',
            parameters=[{"tf_src": "laser_link", "tf_dst": "base_link"}],
            emulate_tty=True
            
        )
    ])