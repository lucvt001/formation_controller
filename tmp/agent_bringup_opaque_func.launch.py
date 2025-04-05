import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

launch_args = [
    DeclareLaunchArgument('use_gps', default_value='True', description='gps or filter. "True" for gps (leaders), "False" for followers (filter).'),
    DeclareLaunchArgument('is_unity_sam', default_value='True', description='"True" if running on Unity with SAM, "False" otherwise.'),
]

def launch_setup(context):

    # Overall configuration file for all nodes here
    config = os.path.join(
        get_package_share_directory('formation_controller'),
        'config',
        'overall_params.yaml'
    )

    # Distinguish leaders and followers
    use_gps = LaunchConfiguration('use_gps').perform(context)

    # Distinguish if running on Unity with SAM or not. If with SAM, will need the relay nodes
    is_unity_sam = LaunchConfiguration('is_unity_sam').perform(context)

    # Common node arguments
    pkg = 'formation_controller'

    # Subscribe to gps and heading topic of supreme leader. Broadcast world (utm) -> map, no translation, only rotation
    # Publish origin gps to /NS/origin_gps so that each agent can calculate its current local position, FLU frame
    origin_pub = Node(
        name='origin_pub',
        executable='origin_pub',
        package=pkg, parameters=[config]
    )

    # Listen to target -> filtered_position transform and publish it to three topics: x, y, z
    # Used for PID control of each axis
    tf_repub = Node(
        name='tf_repub',
        executable='tf_repub',
        package=pkg, parameters=[config]
    )

    # Read the yaml file and broadcast all the static transforms: leader -> agentX
    formation_shape_broadcaster = Node(
        name='formation_shape_broadcaster',
        executable='formation_shape_broadcaster',
        package=pkg,
        parameters=[{
            'yaml_file_path': os.path.join(get_package_share_directory('formation_controller'), 'config', 'formation_shape.yaml'),
            'leader_frame': 'agent0/base_link',
        }]
    )

    # Listen to gps and heading of this agent. Calculate the local position relative to origin_gps
    # And then broadcast the transform world (utm) -> agent
    # Only used for agents on the surface with access to gps (aka leaders)
    gps_heading_to_tf = Node(
        name='gps_heading_to_tf',
        executable='gps_heading_to_tf',
        package=pkg, parameters=[config]
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
                package='position_filter'
            ),
        ]
    )

    # PID servers for followers control scheme
    pid_servers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('formation_controller') + '/launch/pid_servers.launch.py'
        )
    )

    # Required for the controller to work
    differential_value_node = Node(
        name='get_differential_value',
        executable='get_differential_value',
        package=pkg, parameters=[config]
    )

    # Relay nodes for Unity with SAM
    relay_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('formation_controller') + '/launch/relay_nodes.launch.py'
        )
    )

    ld_list = [
        origin_pub, 
        tf_repub, 
        formation_shape_broadcaster, 
        pid_servers, 
        differential_value_node
    ]

    if use_gps == 'True':
        ld_list.append(gps_heading_to_tf)
    elif use_gps == 'False':
        ld_list.append(ukf_filter)
    else:
        raise ValueError("Invalid value for use_gps. Expected 'True' or 'False'.")
    
    if is_unity_sam == 'True':
        ld_list.append(relay_nodes)
    elif is_unity_sam == 'False':
        pass
    else:
        raise ValueError("Invalid value for is_unity_sam. Expected 'True' or 'False'.")

    return ld_list

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
