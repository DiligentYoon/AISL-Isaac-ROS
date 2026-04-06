from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    duration_arg = DeclareLaunchArgument(
        'duration', default_value='3.0',
        description='Plotter data collection duration in seconds')
    save_arg = DeclareLaunchArgument(
        'save', default_value='false',
        description='Save plotter figures as PNG (true/false)')

    nsc_control = Node(
        package='goat',
        executable='nsc_control_node',
        name='nsc_control_node',
        output='screen',
    )

    nsc_plotter = Node(
        package='goat',
        executable='nsc_plotter',
        name='nsc_plotter',
        output='screen',
        arguments=[
            '--duration', LaunchConfiguration('duration'),
            '--save', LaunchConfiguration('save'),
        ],
    )

    return LaunchDescription([
        duration_arg,
        save_arg,
        nsc_control,
        nsc_plotter,
    ])
