#! /usr/bin/env python3
import os
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import re
import math
from tf_transformations import euler_from_quaternion
import random
import time
import yaml
import ament_index_python.packages as packages
import cbs

CUSTOM_GOALS = True # True: read positions from /params/custom_goals.yaml False: Random targets
REPLAN = False # True: Plan and execute continuosly False: Plan and execute once
THRE_ROBOT_ON_TARGET = 0.1 # Threshold to consider that robot has reached a target
TARGETS_RANDOM_POOL_SIZE = 4 # Size of target area in meters (Gazebo squares)
KP = 0.15 # Controller "proportional" gain
MAX_LINEAR_VELOCITY = 1
MAX_ANGULAR_VELOCITY = 0.2
DISCRETIZATION = 2 # Discretization of map (1: one robot per gazebo square, 2: 4 robots per gazebo square)
SHIFT_MAP = 10*DISCRETIZATION # CBS only accepts positive values
CBS_MAP_DIMENSION = 20*DISCRETIZATION

class Planner(Node):
    def __init__(self):
        super().__init__('robot_controller')      
        self.number_of_robots = int(self.count_robot_topics())        
        self.robots = ['robot{}'.format(i) for i in range(self.number_of_robots)]
        self.subscribers = []
        self.robot_publishers = []
        self.positions = [Point() for _ in range(len(self.robots))]
        self.orientations = [float for _ in range(len(self.robots))]
        self.distance_to_target = [float for _ in range(len(self.robots))]
        self.final_goal = [Point() for _ in range(len(self.robots))]
        self.position_received = 0
        self.first_time_planning = True
        self.cbs_time_schedule = 1
        self.number_of_succesfully_executed_plans = 0
        self.obstacles=[500, 500] #Dummy obstacle
        # self.read_obstacle_param()
        self.create_all_subscribers()
        self.create_all_publishers()
        self.generate_new_targets()
        self.get_logger().info(f'New targets acquired: {self.final_goal}')
        
    def count_robot_topics(self):
        topic_list = Node.get_topic_names_and_types(self)
        robot_count = 0
        for item in topic_list:
            if item[0].startswith('/robot') and item[0].endswith('/cmd_vel'):
                robot_count += 1        
        return robot_count
        
    def create_all_subscribers(self):
        for robot_name in self.robots:
            topic = f'/{robot_name}/odom'
            subscriber = self.create_subscription(Odometry, topic, lambda msg, topic=topic: self.position_callback(msg, topic), 10)
            subscriber.robot_name = robot_name
            self.subscribers.append(subscriber)
    
    def create_all_publishers(self):
        for robot_name in self.robots:
            topic = f'/{robot_name}/cmd_vel'
            publisher = self.create_publisher(Twist, topic, 10)
            self.robot_publishers.append(publisher)

    def position_callback(self, msg, topic):
        robot_index = re.search(r'/robot(\d+)/odom', topic).group(1)
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        (_, _, theta)  = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.positions[int(robot_index)] = Point(x=position.x, y=position.y, z=position.z)
        self.orientations[int(robot_index)] = theta
        
        # Compute a new control input for every update in position
        self.drive_robots_to_cbs_waypoints()
        
    def all_robots_arrived_cbs_final_waypoint(self):
        # Check if all robots arrived in cbs last waypoint
        number_of_robots_in_target = 0
        cbs_last_waypoints = []
        for robot_index, _ in enumerate(self.target_waypoints):
            cbs_last_waypoints.append(self.target_waypoints[robot_index][-1])
        for robot_index, _ in enumerate(self.robots):
            current_position = self.positions[robot_index]
            self.distance_to_target[int(robot_index)] = self.get_distance((cbs_last_waypoints[int(robot_index)].x - SHIFT_MAP)/DISCRETIZATION - current_position.x, (cbs_last_waypoints[int(robot_index)].y - SHIFT_MAP)/DISCRETIZATION - current_position.y)
            if self.distance_to_target[int(robot_index)] < THRE_ROBOT_ON_TARGET:
                number_of_robots_in_target += 1 
        if int(number_of_robots_in_target) == int(len(self.robots)): # All robots on last waypoint
            self.halt_robots()
            self.number_of_succesfully_executed_plans += 1
            self.get_logger().warning(f'Number of succesfully executed plans: {self.number_of_succesfully_executed_plans}')
            self.cbs_time_schedule = 1 # Plan again
            return True
        else:
            return False        
    
    def all_robots_arrived_in_waypoints(self):
        # Check if all robots arrived in target positions
        number_of_robots_in_waypoints = 0
        for robot_index, _ in enumerate(self.robots):
            current_position = self.positions[robot_index]
            target_waypoint  = self.get_next_target_waypoint(robot_index)
            self.distance_to_target[int(robot_index)] = self.get_distance((target_waypoint.x -SHIFT_MAP)/DISCRETIZATION - current_position.x, (target_waypoint.y-SHIFT_MAP)/DISCRETIZATION - current_position.y)
            if self.distance_to_target[int(robot_index)] < THRE_ROBOT_ON_TARGET:
                number_of_robots_in_waypoints += 1 
        if int(number_of_robots_in_waypoints) == int(len(self.robots)):
            self.get_logger().info(f'Robots Arrived in waypoint: {self.cbs_time_schedule}')
            return True
        else:
            return False
        
    def drive_robots_to_cbs_waypoints(self):
        '''
        Drive robots to the waypoints that are solution of the CBS planner. 
        When the target is reached, another random target is acquired and a new plan is generated
        Do this continuously
        '''
        if self.all_positions_received():
            self.call_cbs_planner()
                           
        if self.first_time_planning == False: # Planner was already called at least once
            if self.all_robots_arrived_cbs_final_waypoint(): # all robots arrived -> new targets -> call planner
                if REPLAN == True:
                    self.get_logger().info('All robots arrived on targets')
                    self.get_logger().info('Acquiring new targets')
                    self.generate_new_targets()
                    self.call_cbs_planner()
                else:
                    self.get_logger().warning(f'Plan Executed sucessfully!')
                    sys.exit()
            else: #Drive robots to next waypoint
                if self.all_robots_arrived_in_waypoints():
                    if self.cbs_time_schedule < max(self.max_cbs_times):
                        self.cbs_time_schedule += 1 # Next time in CBS schedule
                        self.get_logger().info(f'Going to waypoint: {self.cbs_time_schedule} | of total: {max(self.max_cbs_times)}')
                    
                for robot_index, robot_name in enumerate(self.robots):  
                    target_waypoint = self.get_next_target_waypoint(robot_index)
                    current_orientation = self.orientations[robot_index]             
                    # Validate current_orientation
                    if isinstance(current_orientation, (int, float)):
                        current_orientation = float(current_orientation)
                    else:
                        error_msg = "Invalid current_orientation: " + str(current_orientation)
                        continue
                    self.command_robot(robot_index, target_waypoint, current_orientation)
                    
    def command_robot(self, robot_index, target_waypoint, current_orientation):
        d = 0.1 # Virtual point outside robot center (avoid mathematical errors in the feedback linearization controller)
        current_position = self.positions[robot_index]
        cmd_vel = Twist()
        try: # Feedback linearization controller
            target_x_scaled = (target_waypoint.x - SHIFT_MAP)/DISCRETIZATION
            target_y_scaled = (target_waypoint.y - SHIFT_MAP)/DISCRETIZATION
            error_x = target_x_scaled - current_position.x
            error_y = target_y_scaled - current_position.y
            cmd_vel.linear.x = (error_x) * math.cos(current_orientation) + (error_y) * math.sin(current_orientation)
            cmd_vel.angular.z = -(error_x) * math.sin(current_orientation) / d + (error_y) * math.cos(current_orientation) / d
            cmd_vel.linear.x = KP * cmd_vel.linear.x
            cmd_vel.angular.z = KP * cmd_vel.angular.z
            # Limit the velocities
            cmd_vel.linear.x = min(cmd_vel.linear.x, MAX_LINEAR_VELOCITY)
            cmd_vel.angular.z = min(cmd_vel.angular.z, MAX_ANGULAR_VELOCITY)
            self.robot_publishers[int(robot_index)].publish(cmd_vel)
        except Exception as e:
            error_msg = "Error occurred during computation: " + str(e)
            self.get_logger().error(error_msg)             

    def halt_robots(self):
        cmd_vel = Twist()
        for robot_index, _ in enumerate(self.robots):
            try:
                cmd_vel.linear.x = float(0)
                cmd_vel.linear.y = float(0)
                cmd_vel.angular.x = float(0)
                cmd_vel.angular.y = float(0)
                cmd_vel.angular.z = float(0)
                self.robot_publishers[int(robot_index)].publish(cmd_vel)
            except Exception as e:
                error_msg = "Error halting robots: " + str(e)
                self.get_logger().error(error_msg)     

    def get_next_target_waypoint(self, robot_index):
        if len(self.target_waypoints[robot_index]) > self.cbs_time_schedule: # While new CBS waypoints exists -> Go to them
            target_waypoint = self.target_waypoints[robot_index][self.cbs_time_schedule]
        else:
            target_waypoint = self.target_waypoints[robot_index][self.max_cbs_times[robot_index]] # Reached its final waypoint -> Stay put
        return target_waypoint

    def generate_new_targets(self):
        if CUSTOM_GOALS == False:
            self.generate_new_random_targets()
        else:
            self.get_logger().info(f'custom goals')
            self.read_and_set_custom_goals()

    def generate_new_random_targets(self):
        for robot_index, _ in enumerate(self.robots):
            self.final_goal[int(robot_index)] = Point(x=float(random.randint(-TARGETS_RANDOM_POOL_SIZE*DISCRETIZATION, TARGETS_RANDOM_POOL_SIZE*DISCRETIZATION)), y=float(random.randint(-TARGETS_RANDOM_POOL_SIZE*DISCRETIZATION, TARGETS_RANDOM_POOL_SIZE*DISCRETIZATION)), z=0.01) # Random target positions
        while self.check_equal_points(self.final_goal) == True: # Unique targets
            self.generate_new_random_targets()     
        # self.resolve_obstacle_conflicts()

    def check_equal_points(self, points):
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                if points[i] == points[j]:
                    return True  # Found equal points
        return False  # No equal points found

    def write_data_to_yaml(self):       
        input_filename = 'scripts/params/cbs_input.yaml'
        filename = self.get_full_filename(input_filename)
        self.get_logger().info(f'cbs_input path: {filename}')
        
        data = {'robots': [],
                "map": {
                "dimensions": [CBS_MAP_DIMENSION, CBS_MAP_DIMENSION],
                "obstacles": self.obstacles}}
        for robot_index, robot_name in enumerate(self.robots):
            start_x = int(round(self.positions[int(robot_index)].x*DISCRETIZATION))
            start_y = int(round(self.positions[int(robot_index)].y*DISCRETIZATION))
            goal_x = int(round(self.final_goal[int(robot_index)].x))
            goal_y = int(round(self.final_goal[int(robot_index)].y))
            agent_data = {'start': [(start_x + SHIFT_MAP), (start_y + SHIFT_MAP)], 'goal': [goal_x + SHIFT_MAP, goal_y + SHIFT_MAP], 'name': robot_name}
            data['robots'].append(agent_data)

        with open(filename, 'w') as file:
            yaml.dump(data, file, default_flow_style=None, indent=4)
            
    def read_obstacle_param(self):
        input_filename = 'scripts/params/custom_obstacles.yaml'
        filename = self.get_full_filename(input_filename)
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        self.obstacles=[]
        for data in data['obstacles']:
            self.get_logger().info(f"Data obstacles: {data}")
            obst = data['obstacle']
            self.obstacles.append([int(obst[0]*DISCRETIZATION + SHIFT_MAP), int(obst[1]*DISCRETIZATION + SHIFT_MAP)])
        self.get_logger().info(f"self.obstacles: {self.obstacles}")
    
    def read_and_set_custom_goals(self):
        input_filename = 'scripts/params/custom_goals.yaml'
        filename = self.get_full_filename(input_filename)
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        data_robots = data['robots']
        size_custom_inputs = len(data_robots)        
        for robot_index, robot_data in enumerate(data_robots):
            if robot_index < self.number_of_robots:
                goal = robot_data['goal']
                self.final_goal[int(robot_index)].x = float(round(goal[0]))
                self.final_goal[int(robot_index)].y = float(round(goal[1]))
        self.resolve_goal_conflicts()
        # self.resolve_obstacle_conflicts()
                   
    def update_goal(self, robot_id, new_goal):
        self.final_goal[robot_id] = new_goal
        self.get_logger().info(f"Updated Goal for robot-{robot_id}: [{new_goal.x}, {new_goal.y}]")

    def resolve_goal_conflicts(self):
        conflict = True
        while conflict:
            conflict = False
            for robot_i, goal_i in enumerate(self.final_goal):
                for robot_j, goal_j in enumerate(self.final_goal):
                    if robot_i > robot_j and goal_i == goal_j:
                        self.get_logger().info(f"Same goal for robot-{robot_i} and robot-{robot_j}")
                        new_goal = self.generate_new_goal(goal_i)
                        self.update_goal(robot_i, new_goal)
                        conflict = True
                        
    def resolve_obstacle_conflicts(self):
        conflict = True
        while conflict:
            conflict = False
            for robot_i, goal_i in enumerate(self.final_goal):
                for obstacle_i in self.obstacles:
                    if goal_i == obstacle_i:
                        self.get_logger().info(f"Robot-{robot_i} goal is inside obstacle-{obstacle_i}")
                        new_goal = self.generate_new_goal(goal_i)
                        self.update_goal(robot_i, new_goal)
                        conflict = True

    def generate_new_goal(self, old_goal):
        new_goal = Point(x=old_goal.x + random.randint(-1, 1),
                         y=old_goal.y + random.randint(-1, 1),
                         z=0.0)
        return new_goal
                
    def get_data_from_yaml(self):
        output_filename = 'scripts/params/cbs_output.yaml'
        filename = self.get_full_filename(output_filename)
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        if not data:
            self.get_logger().info(f'Solution not found!')
            time.sleep(2)
            self.first_time_planning = True
            self.generate_new_targets()
            return
        else:
            status = data["status"]
            if status == 0:
                self.get_logger().info(f'Solution not found!')
                time.sleep(2)
                self.first_time_planning = True
                self.generate_new_targets()
                return
            else:
                self.get_logger().info(f'Solution found!')

            schedule = data['schedule']
            self.target_waypoints = [[] for _ in range(len(self.robots))]
            self.max_cbs_times = [int for _ in range(len(self.robots))]
            for robot, waypoints in schedule.items():
                robot_index = re.search(r'robot(\d+)', robot).group(1)
                for wp in waypoints:
                    waypoint = wp.items()
                    for item in waypoint:
                        key, value = item                    
                        if key == 'x':
                            x_value = value
                        elif key == 'y':
                            y_value = value
                        elif key == 't':
                            t_value = value
                            self.max_cbs_times[int(robot_index)] = int(t_value)
                    self.target_waypoints[int(robot_index)].append(Point(x=float(x_value), y=float(y_value), z=0.01))
            self.get_logger().info(f'Max number of waypoints to execute: {max(self.max_cbs_times)}')
    
    def get_full_filename(self, param_filename):   
        package_path = packages.get_package_share_directory('planning')
        self.get_logger().info(f'package path: {package_path}')
        substring = 'install/planning/share/'
        filename = os.path.join(package_path, param_filename)
        filename = filename.replace(substring, '')
        return filename
    
    def all_positions_received(self):
        if self.first_time_planning == True:
            self.position_received = 0
            for robot_index, _ in enumerate(self.robots):
                current_position = self.positions[robot_index]
                if current_position.x != 0 and current_position.y != 0:
                    self.position_received += 1                    
            if self.position_received == len(self.robots): # All positions received
                self.get_logger().info('All positions received')
                self.first_time_planning = False
                return True
        return False
        
    def verify_initial_robots_positions(self):
        for robot_i, _ in enumerate(self.robots):
            for robot_j, _ in enumerate(self.robots):
                if robot_i != robot_j:
                    dist = self.get_distance((self.positions[robot_i].x - self.positions[robot_j].x), (self.positions[robot_i].y - self.positions[robot_j].y))
                    if dist < 0.8*(1/DISCRETIZATION):
                        self.get_logger().info('Robots are to close, CBS will not find a solution')
                        
    def call_cbs_planner(self):
        self.verify_initial_robots_positions()
        self.halt_robots()
        self.get_logger().info(f'Conflict Based Search Planning')
        self.write_data_to_yaml()
        time.sleep(1)
        self.get_logger().info(f'Searching for solution...')
        cbs.main()
        time.sleep(1)
        self.get_data_from_yaml()
        
    def get_distance(self, dx, dy):
        return math.sqrt(dx*dx + dy*dy)

def main(args=None):
    rclpy.init(args=args)
    planning_and_control = Planner()
    rclpy.spin(planning_and_control)
    planning_and_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()