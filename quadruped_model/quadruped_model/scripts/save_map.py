#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
from ament_index_python.packages import get_package_share_path


class OctomapSaver(Node):

    def __init__(self):
        super().__init__('octomap_saver')
        # self.save_dirr = str(get_package_share_path('quadruped_model')) + "/maps/"
        self.save_dirr = "/home/mert/quadruped_gazebo/src/quadruped_model/maps/"

        self.create_subscription(
            Octomap,
            'octomap_full',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        print(msg, "----------------\n\n")

        normalized_data = [i + 128 for i in msg.data]

        with open(self.save_dirr + "map_quadruped.bt", "wb") as f:
            f.write(bytes(normalized_data))

        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()

    octomap_saver = OctomapSaver()

    rclpy.spin(octomap_saver)

    octomap_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()