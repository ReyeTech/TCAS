#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import re
import math
from tf_transformations import euler_from_quaternion
import random
import time

THRE_ROBOT_ON_TARGET = 0.1 # Thredshould to consider that robot has reached a target
TARGETS_RANDOM_POOL_SIZE = 2 # Size of target area
KP = 0.05 # Controller proportional gain
MAX_LINEAR_VELOCITY = 0.8
MAX_ANGULAR_VELOCITY = 0.2

class Planner(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        #TODO: automate number of robots
        self.robots = ['robot0', 'robot1', 'robot2']
        self.subscribers = []
        self.robot_publishers = []
        self.positions = [Point() for _ in range(len(self.robots))]
        self.orientations = [float for _ in range(len(self.robots))]
        self.distance_to_target = [float for _ in range(len(self.robots))]
        self.target_positions = [Point() for _ in range(len(self.robots))]
        for robot_index, robot_name in enumerate(self.robots):
            self.target_positions[int(robot_index)] = Point(x=float(random.randint(-TARGETS_RANDOM_POOL_SIZE, TARGETS_RANDOM_POOL_SIZE)), y=float(random.randint(-TARGETS_RANDOM_POOL_SIZE, TARGETS_RANDOM_POOL_SIZE)), z=0.01) # Random target positions
        self.create_all_publishers()
        self.create_all_subscribers()
    
    def create_all_publishers(self):
        for robot_name in self.robots:
            topic = f'/{robot_name}/odom'
            subscriber = self.create_subscription(
                Odometry,
                topic,
                lambda msg, topic=topic: self.position_callback(msg, topic),
                10
            )
            subscriber.robot_name = robot_name
            self.subscribers.append(subscriber)
    
    def create_all_subscribers(self):
        for robot_name in self.robots:
            topic = f'/{robot_name}/cmd_vel'
            publisher = self.create_publisher(
                Twist,
                topic,
                10
            )
            self.robot_publishers.append(publisher)

    def position_callback(self, msg, topic):
        robot_index = re.search(r'/robot(\d+)/odom', topic).group(1)
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        (roll, pitch, theta)  = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.positions[int(robot_index)] = Point(x=position.x, y=position.y, z=position.z)
        self.orientations[int(robot_index)] = theta
        
        # Compute a new control input for every update in position
        self.drive_robots_to_random_positions()
        
    def drive_robots_to_random_positions(self):
        while not self.positions or not self.orientations:
            self.get_logger().info('Waiting for current position...')
            rclpy.spin_once(self)
        
        # Check if all robots arrived in target positions
        number_of_robots_in_target = 0
        for robot_index, _ in enumerate(self.robots):
            current_position = self.positions[robot_index]
            current_orientation = self.orientations[robot_index]
            self.distance_to_target[int(robot_index)] = self.get_distance_to_target(self.target_positions[int(robot_index)].x - current_position.x, self.target_positions[int(robot_index)].y - current_position.y)
            if self.distance_to_target[int(robot_index)] < THRE_ROBOT_ON_TARGET:
                number_of_robots_in_target += 1 
        
        if int(number_of_robots_in_target) == int(len(self.robots)): # all robots arrived -> new targets
            for robot_index, _ in enumerate(self.robots):
                self.get_logger().info('All robots arrived on targets')
                self.get_logger().info('Acquiring new target')
                self.target_positions[int(robot_index)] = Point(x=float(random.randint(-TARGETS_RANDOM_POOL_SIZE, TARGETS_RANDOM_POOL_SIZE)), y=float(random.randint(-TARGETS_RANDOM_POOL_SIZE, TARGETS_RANDOM_POOL_SIZE)), z=0.01) # Random target positions
                time.sleep(2)

        rate = self.create_rate(1)
        
        d = 0.1 # Virtual point outside robot center (avoid mathematical errors in the feedback linearization controller)
        for robot_index, robot_name in enumerate(self.robots):
            current_position = self.positions[robot_index]
            current_orientation = self.orientations[robot_index]
            target_position = self.target_positions[robot_index]
            cmd_vel = Twist()

            # Validate current_orientation
            if isinstance(current_orientation, (int, float)):
                current_orientation = float(current_orientation)
            else:
                error_msg = "Invalid current_orientation: " + str(current_orientation)
                self.get_logger().error(error_msg)
                continue
            try: # Feedback linearization controller
                cmd_vel.linear.x = (target_position.x - current_position.x) * math.cos(current_orientation) + (target_position.y - current_position.y) * math.sin(current_orientation)
                cmd_vel.angular.z = -(target_position.x - current_position.x) * math.sin(current_orientation) / d + (target_position.y - current_position.y) * math.cos(current_orientation) / d
                cmd_vel.linear.x = KP * cmd_vel.linear.x
                cmd_vel.angular.z = KP * cmd_vel.angular.z
                # Limit the velocities
                cmd_vel.linear.x = min(cmd_vel.linear.x, MAX_LINEAR_VELOCITY)
                cmd_vel.angular.z = min(cmd_vel.angular.z, MAX_ANGULAR_VELOCITY)
                self.robot_publishers[int(robot_index)].publish(cmd_vel)
            except Exception as e:
                error_msg = "Error occurred during computation: " + str(e)
                self.get_logger().error(error_msg)
            
            # self.get_logger().info(f'Robot: {robot_index}, Target: {target_position}, Current: {current_position}')

    def get_distance_to_target(self, dx, dy):
        # Calculate the Euclidean distance between the current position and the target position
        return math.sqrt(dx*dx + dy*dy)

def main(args=None):
    rclpy.init(args=args)
    planning_and_control = Planner()
    rclpy.spin(planning_and_control)
    planning_and_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
