import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

launch_args = [
    DeclareLaunchArgument('ns', description='Namespace for the agent.'),
    DeclareLaunchArgument('use_ukf', description='True to enable UKF, False to disable it.')
]

def generate_launch_description():

    ns = LaunchConfiguration('ns')
    use_ukf = LaunchConfiguration('use_ukf')

    # Relay nodes for SAM
    relay_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sam_thruster_relay'), 'launch', 'relay_nodes.launch.py')
        )
    )
    relay_nodes = GroupAction([PushRosNamespace(ns), relay_nodes])

    follower_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('formation_controller'), 'launch', 'follower_bringup.launch.py')
        ), launch_arguments={
            'ns': ns,
            'use_ukf': use_ukf,
            'rosbag': 'False',
            'have_gps': 'True',
        }.items()
    )

    string_stamped_processing = Node(
        name='string_stamped_processing',
        executable='string_stamped_processing',
        package='tuper_sim_utils',
        namespace=ns,
        output='screen',
        parameters=[{
            'follower_acoustic_topic': 'acoustic/read',
            'leader1_distance_topic': 'leader1/distance',
            'leader2_distance_topic': 'leader2/distance',
            'leader1_msg_topic': 'leader1/msg',
            'leader2_msg_topic': 'leader2/msg'
        }]
    )

    return LaunchDescription([
        *launch_args,
        follower_bringup,
        relay_nodes,
        string_stamped_processing,
    ])