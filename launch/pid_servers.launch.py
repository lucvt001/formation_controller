import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    pid_config = os.path.join(
        get_package_share_directory('formation_controller'),
        'config',
        'pid_params.yaml'
    )

    x_pid = Node(
        name='x_pid',
        executable='pid_server',
        package='formation_controller',
        output='screen',
        parameters=[pid_config]
    )

    y_pid = Node(
        name='y_pid',
        executable='pid_server',
        package='formation_controller',
        output='screen',
        parameters=[pid_config]
    )

    y_delta_pid = Node(
        name='y_delta_pid',
        executable='pid_server',
        package='formation_controller',
        output='screen',
        parameters=[pid_config]
    )

    y_pid_turn = Node(
        name='y_pid_turn',
        executable='pid_server',
        package='formation_controller',
        output='screen',
        parameters=[pid_config]
    )

    y_delta_pid_turn = Node(
        name='y_delta_pid_turn',
        executable='pid_server',
        package='formation_controller',
        output='screen',
        parameters=[pid_config]
    )

    return LaunchDescription([
        TimerAction(period=0.0, actions=[x_pid]),
        TimerAction(period=0.1, actions=[y_pid]),
        TimerAction(period=0.2, actions=[y_delta_pid]),
        TimerAction(period=0.3, actions=[y_pid_turn]),
        TimerAction(period=0.4, actions=[y_delta_pid_turn]),
    ])