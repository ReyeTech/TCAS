#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import re

class Planner(Node):
    def __init__(self):
        super().__init__('robot_position_reader')

        #TODO: automate number of robots
        self.robots = ['robot0', 'robot1', 'robot2']
        self.subscribers = []
        self.positions = [[] for _ in range(len(self.robots))]

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

    def position_callback(self, msg, topic):
        robot_index = re.search(r'/robot(\d+)/odom', topic).group(1)
        position = msg.pose.pose.position
        self.get_logger().info(f'{robot_index} position: x={position.x}, y={position.y}, z={position.z}')
        self.positions[int(robot_index)].append([position.x, position.y, position.z])

def main(args=None):
    rclpy.init(args=args)
    position_reader = Planner()
    rclpy.spin(position_reader)
    position_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
