import os
from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_context import LaunchContext

launch_args = [
    DeclareLaunchArgument('ns', description='Namespace for the agent. Should be agent0, agent1, agent2, etc.'),
    DeclareLaunchArgument('use_ukf', description='True to use UFK result for control, False for GPS (but GPS node is still on for ground truth). Automatically switch between NS/ukf_link and NS/gps_link for control.'),
    DeclareLaunchArgument('rosbag', description='True to start ros2bag record.'),
]

def launch_setup(context: LaunchContext) -> list[Action]:

    # Overall configuration file for most nodes
    config = os.path.join(get_package_share_directory('formation_controller'), 'config', 'overall_params.yaml')

    ns = LaunchConfiguration('ns')
    use_ukf = LaunchConfiguration('use_ukf')
    rosbag = LaunchConfiguration('rosbag')

    if use_ukf.perform(context) == 'True':
        body_frame_to_use = 'NS/ukf_link'
    else:
        body_frame_to_use = 'NS/gps_link'

    # Rosbag
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        condition=IfCondition(PythonExpression([rosbag])),
    )

    # Listen to target -> filtered_position transform and publish it to three topics: x, y, z
    # Used for PID control of each axis
    tf_repub = Node(
        name='tf_repub',
        executable='tf_repub',
        package='formation_controller', 
        parameters=[config, {'child_frame': body_frame_to_use}],
    )

    # Read the yaml file and broadcast all the static transforms: leader -> agentX
    formation_shape_broadcaster = Node(
        name='formation_shape_broadcaster',
        executable='formation_shape_broadcaster',
        package='formation_controller',
        parameters=[{
            'yaml_file_path': os.path.join(get_package_share_directory('formation_controller'), 'config', 'formation_shape.yaml'),
            'leader_frame': 'agent0/gps_link',
        }]
    )

    # Listen to gps and heading of this agent. Calculate the local position relative to origin_gps
    # And then broadcast the transform world (utm) -> agent/gps_link
    # Always run for all agents as a source of ground truth, regardless of whether it actually has access to gps
    gps_heading_to_tf = Node(
        name='gps_heading_to_tf',
        executable='gps_heading_to_tf',
        package='formation_controller', parameters=[config]
    )

    # For the other leader which is also on the surface with access to gps
    # Listen to gps and heading of the supreme leader. Calculate the position of the leader relative to origin_gps
    # And then broadcast the transform world (utm) -> supreme_leader/gps_link
    # If the follower is underwater and does not have access to this info, it should have the ukf node enabled. Else no control:)
    leader_gps_heading_to_tf = Node(
        name='leader_gps_heading_to_tf',
        executable='gps_heading_to_tf',
        package='formation_controller', parameters=[config]
    )

    # Fuse ping_distance1 and ping_distance2 to track x, y, vx, vy of the agent relative to supreme_leader
    # And then broadcast supreme_leader -> agent/ukf_link transform. Orientation is ignored.
    # Only used for agents relying on ping distance (aka followers)
    ukf_filter = Node(
        name='ukf_filter',
        executable='position_filter',
        package='position_filter',
        condition=IfCondition(PythonExpression([use_ukf]))
    )

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

    # Required for the controller to work
    sum_and_scale_node = Node(
        name='sum_and_scale',
        executable='sum_and_scale',
        package='formation_controller', parameters=[config]
    )

    # Last but not least, the main controller node
    bt_planner = Node(
        name='bt_planner',
        executable='bt_planner',
        package='tuper_btcpp', output='screen',
        parameters=[{
            'xml_directory': os.path.join(get_package_share_directory('tuper_btcpp'), 'behavior_trees'),
            'tree_name': 'FollowerTest',
            # 'do_connect_groot2': True,
            # 'btlog_output_folder': os.path.join(get_package_share_directory('tuper_btcpp'), 'btlogs'),
            'loop_rate': 20
        }]
    )

    basic_control_nodes = GroupAction(actions=[
        TimerAction(period=0.0, actions=[pid_servers]),
        TimerAction(period=0.5, actions=[differential_value_node]),
        TimerAction(period=0.6, actions=[sum_and_scale_node]),
    ])

    positioning_nodes = GroupAction(actions=[
        TimerAction(period=0.0, actions=[formation_shape_broadcaster]),
        TimerAction(period=0.1, actions=[ukf_filter]),
        TimerAction(period=0.2, actions=[tf_repub]),
        TimerAction(period=0.3, actions=[gps_heading_to_tf]),
        TimerAction(period=0.4, actions=[leader_gps_heading_to_tf]),
    ])

    return [
        PushRosNamespace(ns.perform(context)),
        TimerAction(period=0.0, actions=[positioning_nodes]),
        TimerAction(period=1.0, actions=[basic_control_nodes]),
        TimerAction(period=2.0, actions=[bt_planner]),
        TimerAction(period=6.0, actions=[record_bag]),
    ]

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function=launch_setup)
    ])