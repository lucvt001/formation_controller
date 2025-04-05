import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('formation_controller'),
        'config',
        'overall_params.yaml'
    )

    return LaunchDescription([
        Node(
            name='ground_truth_tf2',
            executable='ground_truth_tf2',
            package='formation_controller',
            parameters=[config]
        ),
        
        # Node(
        #     name='fuse_distance_triangulation',
        #     executable='fuse_distance_triangulation',
        #     package='formation_controller',
        #     parameters=[config]
        # ),
        
        # Node(
        #     name='relative_target_position_publisher',
        #     executable='relative_target_position_publisher',
        #     package='formation_controller',
        #     output='screen',
        #     parameters=[config]
        # ),

        Node(
            name='agent2_tf_repub',
            executable='tf_repub',
            package='formation_controller',
            parameters=[config]
        ),

        Node(
            name='agent0_gps_heading_to_tf',
            executable='gps_heading_to_tf',
            package='formation_controller',
            parameters=[config]
        ),

        Node(
            name='get_differential_value',
            executable='get_differential_value',
            package='formation_controller',
            parameters=[config]
        ),

        Node(
            name='formation_shape_broadcaster',
            executable='formation_shape_broadcaster',
            package='formation_controller',
            parameters=[{
                'yaml_file_path': os.path.join(get_package_share_directory('formation_controller'), 'config', 'formation_shape.yaml'),
            }],
        ),
               
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('formation_controller') + '/launch/pid_servers.launch.py'
            )
        ),  

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('formation_controller') + '/launch/relay_nodes.launch.py'
            )
        ),  

        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "leader1/base_link", "supreme_leader"],
        ),

        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "-1.57", "0", "0", "world", "map_gt"],
        ),

    ])
