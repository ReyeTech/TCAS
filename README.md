# Traffic Alert and Collision Avoidance System - Gazebo Simulation

Robot and world models heavily based on: https://bitbucket.org/theconstructcore/box_bot/src/foxy/

## Requirements

ROS2 Foxy

sudo pip3 install transforms3d
sudo apt-get install ros-foxy-tf-transformations

## To launch gazebo world and N robots

In a new terminal:
```sh
source install/setup.bash

ros2 launch box_bot_gazebo multi_box_bot_launch.py
```

## Planning

Subscribe to all robot position via: /{robot_name}/odom topic
Publish to all robot velocity command via: /{robot_name}/cmd_vel topic
In which robot_name is robot0, robot1, robot2, ...

### Continuously planning

variable REPLAN in /planning/scripts/planner.py
True -> Plan and execute continuosly 
False -> Plan and execute once

### To control robots to random goals:

In a new terminal:

```sh
source install/setup.bash

ros2 launch planning planner_launch.py
```

### To set custom goals:

Set CUSTOM_GOALS = True in planning/scripts/planner.py

Modify goals in planning/params/custom_goals.yaml

If goals of more than one robot is the same, robots will pick another goal close to the original one

## Observations

- Multi Agent Cetralized Conflict Search Based Path planning (CBS MAPF) 
- Robots are differential drive (like turtlebots)
- Number of robots is hardcoded in /TCAS/box_bot_description/launch/multi_spawn_robot_launch.py
- A feedback linearization controller was implemented (To move diff drive robots from initial position X,Y to final position X,Y)

## Path Planning Demo

Video accelerated
Robots have to reach random targets. Conflict Based Search solve for collisions. Robots follow collision free trajectories.  

[![PATH PLANNING DEMO 1](https://github.com/ReyeTech/TCAS/blob/3-Multi-agent-path-planning/gazebo_multiagent2.png?raw=true)](https://youtu.be/fz4IjyRInoU)

## Limitations
- Robots wait other robots before going to new positions, this sincronization is be needed for the path planner
- Collisions may occur if size of robots is too big in relation with map discretization ans obstacles sizes (Path Planning is not aware of robot dynamics)
- CBS code allows positive integers only. Workaround: shift all values, run CBS, shift values back