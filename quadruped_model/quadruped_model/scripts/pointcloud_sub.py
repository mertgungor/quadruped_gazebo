#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import pcl_ros
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):


        points = np.array(pc2.read_points_list(msg, skip_nans=True, field_names=("x", "y", "z")))
        points = self.transform_points(points, 60, np.array(
            [0.0, 
             0.0, 
             0.0]))
        
        self.plot(points)

        height_map = np.ones(shape=(11, 17), dtype=np.float32)*-111

        for point in points:
            # -0.5 -> 0  0.5 -> 11

            x = int(point[0] * 10) + 5
            y = int(point[1] * 10) + 8


            try:
                if point[2] > height_map[x, y]:
                    height_map[x, y] = point[2]
            except IndexError:
                pass

        height_map = np.where(height_map == -111, None, height_map)
        print(height_map)
        
        exit(-1)

    def plot(self, data):

        fig = plt.figure()

        ax = fig.add_subplot(111, projection='3d')
        ax.plot(data[:,0], data[:,1], data[:,2], 'o')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_zticks([-50, 0, 50])
        plt.show()

    def transform_points(self, points, angle, dislocation=np.array([0, 0, 0])):
        # create rotation matrix for pitch angle
        pitch = np.deg2rad(angle)

        rotation_matrix = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                                    [0, 1, 0],
                                    [-np.sin(pitch), 0, np.cos(pitch)]])

        return np.dot(points, rotation_matrix) + dislocation
        


def main(args=None):
    rclpy.init(args=args)

    pointcloud_subscriber = PointCloudSubscriber()

    rclpy.spin(pointcloud_subscriber)

    pointcloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()