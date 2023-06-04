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

## To launch path planner and control robots:

In a new terminal:

```sh
source install/setup.bash

ros2 launch planning planner_launch.py
```

## Basic Architecture

Subscribe to all robot position via: /{robot_name}/odom topic
Publish to all robot velocity command via: /{robot_name}/cmd_vel topic
In which robot_name is robot0, robot1, robot2, ...

### Discretization of map and adjustments

The current implementation of CBS only allows positive integers as inputs and outputs. To workround that, we shift the map so that every point that is inside gazebo default area is a positive integer. Also, we shrink/inflate points according to how we discretize the map. This is done before calling CBS and the other way around when reading CBS solution.

![Map discretization](discretization.png)

Recommended values for discretization: 1 or 2. Other values should also work.


### Continuously planning

variable REPLAN in /planning/scripts/planner.py
True -> Plan and execute continuosly 
False -> Plan and execute once

### To set custom goals:

Set CUSTOM_GOALS = True in planning/scripts/planner.py

Modify goals in planning/params/custom_goals.yaml

If goals of more than one robot are at the same positions, robots will pick another goal close to the original one.

## Alarm

The solution of CBS is published to the topic /planning_alarm as a string.

## Observations

- Multi Agent Centralized Conflict Search Based Path planning (CBS MAPF) 
- Path planner reads start positions and goals for the robots and outputs a schedule of positions for robots to follow
- Number of robots is hardcoded in /TCAS/box_bot_description/launch/multi_spawn_robot_launch.py
- Robots are differential drive (like turtlebots)
- A feedback linearization controller was implemented (To move diff drive robots from initial position X,Y to final position X,Y)

## Path Planning Demos

Robots have to reach random or custom targets. Conflict Based Search solve for collisions. Robots follow collision free trajectories.

[OBSTACLE FREE - CUSTOM GOALS - 10 ROBOTS](https://youtu.be/oolDAnwFhWY)

[OBSTACLE FREE - RANDOM GOALS - 15 ROBOTS](https://youtu.be/EQ7SeHiKW7A)

[WITH OBSTACLES - RANDOM GOALS - 8 ROBOTS](https://youtu.be/HCOMOpOvJdI)

[WITH OBSTACLES - RANDOM GOALS - 15 ROBOTS](https://youtu.be/qwxTIXPNZy4)

[WITH MORE OBSTACLES - RANDOM GOALS - 8 ROBOTS](https://youtu.be/rtTe340uOhU)

## Limitations

- Robots wait other robots before going to new positions, this sincronization is be needed for correct execution of the plan
- Collisions may occur if size of robots is too big in relation with map discretization and obstacles sizes (Path Planning is not aware of robot dynamics)
- CBS code allows positive integers only. Workaround used: shift all values, run CBS, shift values back.
- Robots may spawn on top of obstacles which makes the approach invalid. If this happens, relaunch ros2 launch box_bot_gazebo multi_box_bot_launch.py so that they spawn in other positions. (TODO: check this collisions)
- Obstacles have to be added manually both in the world file (box_bot_gazebo/worlds/box_bot_empty.world) and in the file read by the planner(planning/scripts/params/custom_obstacles.yaml)

## References

- Launch files, world files and robot files based on based on: https://bitbucket.org/theconstructcore/box_bot/src/foxy/
- CBS planner implementation: https://github.com/atb033/multi_agent_path_planning/tree/master/centralized/cbs
- CBS planner theory: https://www.sciencedirect.com/science/article/pii/S0004370214001386


## CPP
To run ros2 node in cpp 

ros2 run planning test_ros2

This command runs the ros node which subscribes to one topic and publishes the data in another topic 

## Current State of Logging
sunny@sunny:~/reye/TCAS$ ros2 launch planning planner_cpp_launch.py 
[INFO] [launch]: All log files can be found below /home/sunny/.ros/log/2023-06-05-00-08-16-089597-sunny-48164
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [Planner_cpp-1]: process started with pid [48167]
[Planner_cpp-1] [INFO] [1685903896.876418341] [Planner]: Number of obstacles added : 13738
[Planner_cpp-1] [INFO] [1685903896.882215388] [Planner]: drive robots to cbs waypoints is next
[Planner_cpp-1] [INFO] [1685903896.882253546] [Planner]: Cbs planner done one time
[Planner_cpp-1] [INFO] [1685903896.883294499] [Planner]: drive robots to cbs waypoints is next
[Planner_cpp-1] [INFO] [1685903896.883331068] [Planner]: Cbs planner done one time
[Planner_cpp-1] [INFO] [1685903896.884141334] [Planner]: drive robots to cbs waypoints is next
[Planner_cpp-1] [INFO] [1685903896.884161922] [Planner]: Cbs planner done one time
[Planner_cpp-1] [INFO] [1685903896.888493959] [Planner]: drive robots to cbs waypoints is next
[Planner_cpp-1] [INFO] [1685903896.888542410] [Planner]: Cbs planner done one time
[Planner_cpp-1] [INFO] [1685903896.891117601] [Planner]: drive robots to cbs waypoints is next
[Planner_cpp-1] [INFO] [1685903896.891172886] [Planner]: Cbs planner done one time
[Planner_cpp-1] [INFO] [1685903896.891771628] [Planner]: drive robots to cbs waypoints is next
[Planner_cpp-1] [INFO] [1685903896.891788760] [Planner]: Cbs planner done one time
[Planner_cpp-1] [INFO] [1685903896.891970161] [Planner]: drive robots to cbs waypoints is next
[Planner_cpp-1] [INFO] [1685903896.891984732] [Planner]: All positions received
[Planner_cpp-1] [INFO] [1685903896.891997403] [Planner]: Halt robots being called
[Planner_cpp-1] [INFO] [1685903896.892008333] [Planner]: going to publish velocities
[Planner_cpp-1] [INFO] [1685903896.892018011] [Planner]: size of publisher velocity vector 6
[Planner_cpp-1] [INFO] [1685903896.892027677] [Planner]: publishing velocities
[Planner_cpp-1] [INFO] [1685903896.892056107] [Planner]: going to publish velocities
[Planner_cpp-1] [INFO] [1685903896.892066185] [Planner]: size of publisher velocity vector 6
[Planner_cpp-1] [INFO] [1685903896.892075199] [Planner]: publishing velocities
[Planner_cpp-1] [INFO] [1685903896.892089410] [Planner]: going to publish velocities
[Planner_cpp-1] [INFO] [1685903896.892098416] [Planner]: size of publisher velocity vector 6
[Planner_cpp-1] [INFO] [1685903896.892107386] [Planner]: publishing velocities
[Planner_cpp-1] [INFO] [1685903896.892123231] [Planner]: going to publish velocities
[Planner_cpp-1] [INFO] [1685903896.892132618] [Planner]: size of publisher velocity vector 6
[Planner_cpp-1] [INFO] [1685903896.892141929] [Planner]: publishing velocities
[Planner_cpp-1] [INFO] [1685903896.892157474] [Planner]: going to publish velocities
[Planner_cpp-1] [INFO] [1685903896.892167536] [Planner]: size of publisher velocity vector 6
[Planner_cpp-1] [INFO] [1685903896.892177270] [Planner]: publishing velocities
[Planner_cpp-1] [INFO] [1685903896.892192342] [Planner]: going to publish velocities
[Planner_cpp-1] [INFO] [1685903896.892201856] [Planner]: size of publisher velocity vector 6
[Planner_cpp-1] [INFO] [1685903896.892210909] [Planner]: publishing velocities
[Planner_cpp-1] [INFO] [1685903896.892225426] [Planner]: Conflict Based Search Planning
[Planner_cpp-1] [INFO] [1685903898.109482877] [Planner]: Searching for solution...
[Planner_cpp-1] [INFO] [1685903899.129722943] [Planner]: Solution found!
[Planner_cpp-1] [INFO] [1685903899.129772426] [Planner]: Progressing in get data from yaml 1
[Planner_cpp-1] [INFO] [1685903899.129799586] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.129820421] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.129839015] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.129855863] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.129871917] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.129888461] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.129904049] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.129919663] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.129935740] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.129952104] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.129968765] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.129984085] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130000937] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130018613] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130035976] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130053711] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130070092] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130086462] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130104051] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130156267] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130175505] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130191457] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130207174] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130222690] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130239448] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130255481] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130271756] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130289442] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130306132] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130322733] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130340854] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130358136] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130374913] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130391214] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130407297] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130423997] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130440865] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130456703] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130472079] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130487873] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130504946] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130521361] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130538195] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130557449] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130574248] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130591925] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130608398] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130624984] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130640482] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130657317] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130673324] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130689320] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130705657] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130721569] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130737153] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130753009] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130768436] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130785468] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130803029] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130819081] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130835523] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130851749] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130867407] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130887908] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130908185] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130924153] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130939953] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130956398] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130972157] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.130987721] [Planner]: Progressing in get data from yaml 2
[Planner_cpp-1] [INFO] [1685903899.131003669] [Planner]: Progressing in get data from yaml 2
[ERROR] [Planner_cpp-1]: process has died [pid 48167, exit code -11, cmd '/home/sunny/reye/TCAS/install/planning/lib/planning/Planner_cpp --ros-args -r __node:=Planner'].
