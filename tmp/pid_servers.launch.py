import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pid_config = os.path.join(
        get_package_share_directory('formation_controller'),
        'config',
        'pid_params.yaml'
    )

    return LaunchDescription([
        Node(
            name='x_pid',
            executable='pid_server',
            package='formation_controller',
            output='screen',
            parameters=[pid_config]
        ),
        Node(
            name='y_pid_approach',
            executable='pid_server',
            package='formation_controller',
            output='screen',
            parameters=[pid_config]
        ),
        Node(
            name='y_pid_maintain',
            executable='pid_server',
            package='formation_controller',
            output='screen',
            parameters=[pid_config]
        ),
        Node(
            name='y_pid_turn',
            executable='pid_server',
            package='formation_controller',
            output='screen',
            parameters=[pid_config]
        ),
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'topic_tools', 'transform',
        #         '/follower/core/thruster1_cmd_float', '/follower/core/thruster1_cmd',
        #         'smarc_msgs/msg/ThrusterRPM',
        #         'smarc_msgs.msg.ThrusterRPM(rpm=int(m.data * 1000))',
        #         '--import', 'smarc_msgs', '--wait-for-start'
        #     ],
        #     output='screen'
        # ),
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'topic_tools', 'relay',
        #         '/follower/core/thruster1_cmd', '/follower/core/thruster2_cmd',
        #         '--wait-for-start'
        #     ],
        #     output='screen'
        # ),
        # Node(
        #     package='formation_controller',
        #     executable='thrust_vector_relay',
        #     name='thrust_vector_relay',
        #     output='screen',
        #     parameters=[{
        #         'vertical_angle_topic': '/follower/core/thrust_vector_cmd_vertical',
        #         'horizontal_angle_topic': '/follower/core/thrust_vector_cmd_horizontal',
        #         'thruster_angles_topic': '/follower/core/thrust_vector_cmd'
        #     }]
        # ),
    ])