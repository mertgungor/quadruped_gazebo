#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
from ament_index_python.packages import get_package_share_path


class OctomapReader(Node):

    def __init__(self):
        super().__init__('octomap_saver')
        file_name = "map.bt"
        # self.save_dirr = str(get_package_share_path('quadruped_model')) + "/maps/" + file_name
        self.save_dirr = "/home/mert/quadruped_gazebo/src/quadruped_model/maps/map_quadruped.bt"


        with open(self.save_dirr  , 'rb') as f:
            read_binary_data = f.read()

        read_data = list(read_binary_data)
        read_data = [i - 128 for i in read_data]
        
        self.octomap = Octomap()
        self.octomap.data = read_data
        self.octomap.header.frame_id = "base"
        # self.octomap.header.stamp = self.get_clock().now().to_msg()
        self.octomap.id = "OcTree"
        self.octomap.resolution = 0.05
        self.octomap.binary = False
        

        self.publisher_ = self.create_publisher(Octomap, 'octomap_full', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.publisher_.publish(self.octomap)
        


def main():
    rclpy.init()

    octomap_saver = OctomapReader()

    rclpy.spin(octomap_saver)

    octomap_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()