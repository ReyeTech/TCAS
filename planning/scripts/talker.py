#! /usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        self.timer_ = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        # Create the string message
        msg = String()
        msg.data = 'Hello, world!'
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info('Published message')


def main(args=None):
    rclpy.init(args=args)
    publisher = MyPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
