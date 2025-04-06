import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    agent_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('formation_controller'), 'launch', 'agent_bringup.launch.py')
        ), launch_arguments={
            'ns': 'follower',
            'use_gps': 'True',
            'is_sam': 'True',
            'is_real': 'False',
            'run_rover': 'False',
            'rosbag': 'True',
        }.items()
    )

    return LaunchDescription([
        agent_bringup,
    ])