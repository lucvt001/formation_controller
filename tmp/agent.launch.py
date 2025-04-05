from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ns_arg = DeclareLaunchArgument(
        'ns', default_value='agent0', description='Namespace for the agent. Should be agent0, agent1, agent2 and so on.'
    )

    ns = LaunchConfiguration('ns')

    general_agent_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('formation_controller') + '/launch/vehicle_agnostic_nodes.launch.py'
        )
    )

    agent_nodes = GroupAction(
        actions=[
            PushRosNamespace(ns),
            general_agent_nodes,
        ]
    )

    return LaunchDescription([
        ns_arg,
        agent_nodes,
    ])
