#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('box_bot_gazebo'), 'launch', 'start_world_launch.py')
                )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('box_bot_description'), 'launch', 'multi_spawn_robot_launch.py')
                )
        )
    ])

    return ld
