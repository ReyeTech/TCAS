# Traffic Alert and Collision Avoidance System - Gazebo Simulation

Robot and world models heavily based on: https://bitbucket.org/theconstructcore/box_bot/src/foxy/


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
- A feedback linearization controller was implemented
- Robots wait other robots before going to new positions, this sincronization is be neededfor the path planner
- Collisions may occur if size of robots is too big in relation with map discretization

## Control Demo

Video accelerated 2x
In this control demo, Robots don't collide out of luck.

[![CONTROL DEMO](https://github.com/ReyeTech/TCAS/blob/37609850ab5f0766d37741fe33248968ab12472f/gazebo_multiagent.png)](https://youtu.be/9uNMXVPop8Q)

## Path Planning Demo
