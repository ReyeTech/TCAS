# Multi Agent Gazebo Simulation

Robot and world models heavily based on: https://bitbucket.org/theconstructcore/box_bot/src/foxy/

## To launch gazebo world and N robots

In a new terminal:
```sh
source install/setup.bash

ros2 launch box_bot_gazebo multi_box_bot_launch.py
```

## To control robots to random targets (continuously):

In a new terminal:

```sh
source install/setup.bash

ros2 launch planning planner_launch.py
```

## Observations

- Robots are differential drive
- Number of robots is hardcoded for now
- To change the number of robots manually change the number in /TCAS/planning/scripts/planner.py and in /TCAS/box_bot_description/launch/multi_spawn_robot_launch.py
- Robots wait other robots before going to new positions, this sincronization will be needed when implementing a global path planner
- Collisions may occur (there is no path planner nor collision avoidance)

## Demo

[![CONTROL DEMO](https://img.youtube.com/vi/YOUTUBE_VIDEO_ID_HERE/0.jpg)](https://youtu.be/9uNMXVPop8Q)
