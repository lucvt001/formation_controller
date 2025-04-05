import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Overall configuration file for all nodes here
    config = os.path.join(
        get_package_share_directory('formation_controller'),
        'config',
        'vehicle_agnostic_params.yaml'
    )

    # Namespace for all nodes in this launch file
    ns_arg = DeclareLaunchArgument('ns', default_value='agent0', description='Namespace for the agent. Should be agent0, agent1, agent2 and so on.')
    ns = LaunchConfiguration('ns')

    # Distinguish leaders and followers
    use_gps_arg = DeclareLaunchArgument('use_gps', default_value='False', description='True if the agent has access to GPS, false if it relies on pingers.')
    use_gps = LaunchConfiguration('use_gps')

    # Common node arguments
    pkg = 'formation_controller'

    # Subscribe to gps and heading topic of supreme leader. Broadcast world (utm) -> map, no translation, only rotation
    # Publish origin gps to /NS/origin_gps so that each agent can calculate its current local position, FLU frame
    origin_pub = Node(
        name='origin_pub',
        executable='origin_pub',
        package=pkg, namespace=ns, parameters=[config]
    )

    # Listen to target -> filtered_position transform and publish it to three topics: x, y, z
    # Used for PID control of each axis
    tf_repub = Node(
        name='tf_repub',
        executable='tf_repub',
        package=pkg, namespace=ns, parameters=[config]
    )

    # Read the yaml file and broadcast all the static transforms: supreme_leader -> agentX
    formation_shape_broadcaster = Node(
        name='formation_shape_broadcaster',
        executable='formation_shape_broadcaster',
        package=pkg, namespace=ns,
        parameters=[{
            'yaml_file_path': os.path.join(get_package_share_directory('formation_controller'), 'config', 'formation_shape.yaml'),
        }]
    )

    # Listen to gps and heading of this agent. Calculate the local position relative to origin_gps
    # And then broadcast the transform world (utm) -> agent
    # Only used for agents on the surface with access to gps (aka leaders)
    gps_heading_to_tf = Node(
        name='gps_heading_to_tf',
        executable='gps_heading_to_tf',
        package=pkg, namespace=ns, parameters=[config]
    )

    # Fuse ping_distance1 and ping_distance2 to track x, y, vx, vy of the agent relative to supreme_leader
    # And then broadcast supreme_leader -> agentX transform. Orientation is ignored.
    # Only used for agents relying on ping distance (aka followers)
    ukf_filter = TimerAction(
        period=1.0,  # Delay in seconds
        actions=[
            Node(
                name='ukf_filter',
                executable='position_filter',
                package='position_filter', namespace=ns
            ),
        ]
    )

    # PID servers for followers control scheme
    pid_servers = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory('formation_controller') + '/launch/pid_servers.launch.py'
                )
            ),
            PushRosNamespace(ns),
        ]
    )
    
    ld_list = [ns_arg, use_gps_arg, origin_pub, tf_repub, formation_shape_broadcaster, pid_servers]
    if use_gps == True:
        ld_list.append(gps_heading_to_tf)
    else:
        ld_list.append(ukf_filter)

    return LaunchDescription(ld_list)
