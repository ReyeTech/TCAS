from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='planning',
            executable='Planner_cpp',
            name='robot_controller',
            output='screen',
            arguments=[]),
    ])