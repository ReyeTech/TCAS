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

## To control robots to random targets (continuously):

Subscribe to all robot position via: /{robot_name}/odom topic
Publish to all robot velocity command via: /{robot_name}/cmd_vel topic
In which robot_name is robot0, robot1, robot2, ...


In a new terminal:

```sh
source install/setup.bash

ros2 launch planning planner_launch.py
```

## Observations

- Multi Agent Cetralized Conflict Search Based Path planning (CBS MAPF) 
- Robots are differential drive (like turtlebots)
- Number of robots is hardcoded in /TCAS/box_bot_description/launch/multi_spawn_robot_launch.py
- A feedback linearization controller was implemented (To move diff drive robots from X,Y to X,Y)
- Robots wait other robots before going to new positions, this sincronization is be needed for the path planner
- Collisions may occur if size of robots is too big in relation with map discretization ans obstacles sizes (Path Planning is not aware of robot dynamics)

## Path Planning Demo

Video accelerated 5x
Robots have to reach random targets. Conflict Based Search solve for collisions. Robots follow collision free trajectories.  

[![PATH PLANNING DEMO 1](https://github.com/ReyeTech/TCAS/blob/3-Multi-agent-path-planning/gazebo_multiagent2.png?raw=true)](https://youtu.be/fz4IjyRInoU)

