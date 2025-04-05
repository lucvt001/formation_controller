from tkinter.font import names
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node

def generate_launch_description():

    namespace = 'agent0'

    general_agent_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('formation_controller') + '/launch/agent_bringup.launch.py'
        )
    )

    agent0_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            general_agent_nodes,
        ]
    )

    return LaunchDescription([
        agent0_nodes,
    ])
