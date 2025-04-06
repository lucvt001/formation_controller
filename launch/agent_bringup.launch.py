from ast import In
import os
from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_context import LaunchContext

launch_args = [
    DeclareLaunchArgument('ns', default_value='follower', description='Namespace for the agent. Should be agent0, agent1, agent2, etc.'),
    DeclareLaunchArgument('use_gps', default_value='True', description='gps or filter. True for gps (leaders), False for followers (filter).'),
    DeclareLaunchArgument('is_sam', default_value='False', description='True if running on SAM, False otherwise. Affect the launch of relay nodes.'),
    DeclareLaunchArgument('is_real', default_value='True', description='True if running on real robot, False otherwise. If sim, it will reuse some nodes from leader.'),
    DeclareLaunchArgument('rosbag', default_value='True', description='True to start ros2bag record.'),
]

def launch_setup(context: LaunchContext) -> list[Action]:

    # Overall configuration file for most nodes
    config = os.path.join(get_package_share_directory('formation_controller'), 'config', 'overall_params.yaml')

    ns = LaunchConfiguration('ns')
    use_gps = LaunchConfiguration('use_gps')
    is_sam = LaunchConfiguration('is_sam')
    is_real = LaunchConfiguration('is_real')
    rosbag = LaunchConfiguration('rosbag')

    # Launch the rover node
    rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arduagent'), 'launch', 'rover_bringup.launch.py')
        ), condition=UnlessCondition(PythonExpression([is_real, ' and not ', is_sam]))
    )

    # Rosbag
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        condition=IfCondition(PythonExpression([rosbag])),
    )

    # Subscribe to gps and heading topic of supreme leader. Broadcast world (utm) -> map, no translation, only rotation
    # Publish origin gps to /NS/origin_gps so that each agent can calculate its current local position, FLU frame
    origin_pub = Node(
        name='origin_pub',
        executable='origin_pub',
        package='formation_controller', parameters=[config]
    )

    # Listen to target -> filtered_position transform and publish it to three topics: x, y, z
    # Used for PID control of each axis
    tf_repub = Node(
        name='tf_repub',
        executable='tf_repub',
        package='formation_controller', parameters=[config]
    )

    # Read the yaml file and broadcast all the static transforms: leader -> agentX
    formation_shape_broadcaster = Node(
        name='formation_shape_broadcaster',
        executable='formation_shape_broadcaster',
        package='formation_controller',
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
        package='formation_controller', parameters=[config],
        condition=IfCondition(PythonExpression([use_gps]))
    )

    # For the other leader which is also on the surface with access to gps
    # Listen to gps and heading of the supreme leader. Calculate the position of the leader relative to origin_gps
    # And then broadcast the transform world (utm) -> supreme_leader
    # In simulation, you don't need this node because the supreme leader would already have its own gps_heading_to_tf node
    # Only needed for real robot bcz the other follower is not ros-communicable with the supreme leader
    leader_gps_heading_to_tf = Node(
        name='leader_gps_heading_to_tf',
        executable='gps_heading_to_tf',
        package='formation_controller', parameters=[config],
        condition=IfCondition(PythonExpression([use_gps, ' and ', is_real]))
    )

    # Fuse ping_distance1 and ping_distance2 to track x, y, vx, vy of the agent relative to supreme_leader
    # And then broadcast supreme_leader -> agentX transform. Orientation is ignored.
    # Only used for agents relying on ping distance (aka followers)
    ukf_filter = Node(
        name='ukf_filter',
        executable='position_filter',
        package='position_filter',
        condition=UnlessCondition(PythonExpression([use_gps]))
    )
    ukf_filter = TimerAction(period=1.0, actions=[ukf_filter])

    # PID servers for followers control scheme
    pid_servers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('formation_controller'), 'launch', 'pid_servers.launch.py')
        )
    )

    # Required for the controller to work
    differential_value_node = Node(
        name='get_differential_value',
        executable='get_differential_value',
        package='formation_controller', parameters=[config]
    )

    # Relay nodes for SAM
    relay_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sam_thruster_relay'), 'launch', 'relay_nodes.launch.py')
        ), condition=IfCondition(PythonExpression([is_sam]))
    )

    # Last but not least, the main controller node
    bt_planner = Node(
        name='bt_planner',
        executable='bt_planner',
        package='tuper_btcpp', output='screen',
        parameters=[{
            'xml_directory': os.path.join(get_package_share_directory('tuper_btcpp'), 'behavior_trees'),
            'tree_name': 'FollowerMainTree',
            # 'do_connect_groot2': True,
            # 'btlog_output_folder': os.path.join(get_package_share_directory('tuper_btcpp'), 'btlogs'),
            'loop_rate': 20
        }]
    )
    bt_planner = TimerAction(period=4.0, actions=[bt_planner])

    return [
        PushRosNamespace(ns.perform(context)),
        rover,
        record_bag,
        origin_pub,
        tf_repub,
        formation_shape_broadcaster,
        gps_heading_to_tf,
        leader_gps_heading_to_tf,
        ukf_filter,
        pid_servers,
        differential_value_node,
        relay_nodes,
        bt_planner
    ]

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function=launch_setup)
    ])