from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='box_bot_planning',
            executable='talker.py',
            output='screen',
            arguments=[]),
    ])