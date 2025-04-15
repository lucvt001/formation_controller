import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Relay nodes for SAM
    relay_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sam_thruster_relay'), 'launch', 'relay_nodes.launch.py')
        )
    )

    follower_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('formation_controller'), 'launch', 'follower_bringup.launch.py')
        ), launch_arguments={
            'ns': 'follower',
            'use_ukf': 'False',
            'rosbag': 'False',
        }.items()
    )

    return LaunchDescription([
        follower_bringup,
        relay_nodes,
    ])