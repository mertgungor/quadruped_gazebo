#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
from std_msgs.msg import Header

class OctomapPublisher(Node):
    def __init__(self, octomap_file, topic):
        super().__init__('octomap_publisher')
        self.publisher_ = self.create_publisher(Octomap, topic, 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # Read the Binary OctoMap file as a binary string
        with open(octomap_file, 'rb') as f:
            self.octomap_data = f.read()

        # Create the Octomap message directly from the binary string
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'odom'
        self.octomap_msg = Octomap(header=header, binary=False, id='OcTree', resolution=0.05, data=self.octomap_data)
        print(self.octomap_msg)
        
        # Publish the Octomap message
        self.publisher_.publish(self.octomap_msg)

    def timer_callback(self):
        self.publisher_.publish(self.octomap_msg)


def main(octomap_file, topic):
    rclpy.init()
    octomap_publisher = OctomapPublisher(octomap_file, topic)
    rclpy.spin(octomap_publisher)
    octomap_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    octomap_file = '/home/mert/quadruped_gazebo/src/quadruped_model/maps/map_bigger.bt'
    topic = 'octomap'
    main(octomap_file, topic)
