#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
import random

SPAWN_AREA = 4
NUMBER_OF_ROBOTS = 6

def gen_robot_list(number_of_robots):
    robots = []
    coordinates = set()  # Set to store unique coordinates

    for i in range(number_of_robots):
        while True:
            x = random.randint(-SPAWN_AREA, SPAWN_AREA)
            y = random.randint(-SPAWN_AREA, SPAWN_AREA)

            # Check if the coordinates are already taken
            if (x, y) not in coordinates:
                break

        coordinates.add((x, y))
        robot_name = "robot" + str(i)
        robots.append({'name': robot_name, 'x_pose': x, 'y_pose': y, 'z_pose': 0.01})

    return robots

def generate_launch_description():

    urdf = os.path.join(get_package_share_directory('box_bot_description'), 'robot/', 'box_bot_v2.urdf')
    pkg_box_bot_description = get_package_share_directory('box_bot_description')
    assert os.path.exists(urdf), "Thebox_bot.urdf doesnt exist in "+str(urdf)

    # Names and poses of the robots
    robots = gen_robot_list(NUMBER_OF_ROBOTS)

    # We create the list of spawn robots commands
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_box_bot_description, 'launch',
                                                           'spawn_box_bot_launch.py')),
                launch_arguments={
                                  'robot_urdf': urdf,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'],
                                  'robot_namespace': robot['name']
                                  }.items()))

    # Create the launch description and populate
    ld = LaunchDescription()
    
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld