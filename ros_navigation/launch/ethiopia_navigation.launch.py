import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ethiopia_navigation')
    world_file = os.path.join(pkg_share, 'worlds', 'ethiopia.world')

    install_dir = os.path.join(pkg_share, '..', '..', '..')
    controller_path = os.path.abspath(
        os.path.join(install_dir, 'ethiopia_navigation', 'bin', 'ethiopia_controller')
    )

    start_arg = DeclareLaunchArgument(
        'start', default_value='Addis Ababa',
        description='Starting city',
    )
    goal_arg = DeclareLaunchArgument(
        'goal', default_value='Harar',
        description='Goal city',
    )

    return LaunchDescription([
        start_arg,
        goal_arg,

        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen',
            name='gazebo_sim',
        ),

        TimerAction(period=2.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
                output='screen',
                name='cmd_vel_bridge',
            ),
        ]),

        TimerAction(period=2.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
                output='screen',
                name='odom_bridge',
            ),
        ]),

        TimerAction(period=7.0, actions=[
            ExecuteProcess(
                cmd=[
                    controller_path,
                    '--start', LaunchConfiguration('start'),
                    '--goal', LaunchConfiguration('goal'),
                ],
                output='screen',
                name='ethiopia_controller',
            ),
        ]),
    ])
