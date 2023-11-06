#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2
import ros2_numpy

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.point_cloud_callback,
            10)
        
    def rgb_to_hsv(r, g, b):
        r, g, b = r / 255.0, g / 255.0, b / 255.0
        mx = max(r, g, b)
        mn = min(r, g, b)
        df = mx - mn
        if mx == mn:
            h = 0
        elif mx == r:
            h = (60 * ((g - b) / df) + 360) % 360
        elif mx == g:
            h = (60 * ((b - r) / df) + 120) % 360
        elif mx == b:
            h = (60 * ((r - g) / df) + 240) % 360
        if mx == 0:
            s = 0
        else:
            s = df / mx
        v = mx
        return h, s, v

    def point_cloud_callback(self, point_cloud):
        # Convert point cloud to a dictionary
        cloud_dict = ros2_numpy.point_cloud2.point_cloud2_to_array(point_cloud)
        
        xyz_array = cloud_dict['xyz']
        #print(xyz_array)
        rgb_array = cloud_dict['rgb']
        #print(rgb_array)
        
        # Initialize an array to hold the XYZ coordinates of blue points
        blue_points_xyz = []

        # Iterate over the RGB values and corresponding XYZ coordinates
        count = 0
        for (xyz, rgb) in zip(xyz_array, rgb_array):
            # Convert the RGB to HSV
            h, s, v = rgb_to_hsv(*rgb)
            #print(np.array([[hsv]]))
            # Define the lower and upper bounds for the shade of blue
            lower_blue = np.array([(200/2), (80/100)*255, (30/100)*255])
            upper_blue = np.array([(240/2), (100/100)*255, (100/100)*255])
            
            # Check if the color is within the blue range
            if (lower_blue[0] <= hsv[0] <= upper_blue[0] and 
                lower_blue[1] <= hsv[1] <= upper_blue[1] and 
                lower_blue[2] <= hsv[2] <= upper_blue[2]):
                # Append the XYZ coordinates to the blue_points_xyz array
                blue_points_xyz.append(xyz)
                count += 1
        
        # Calculate the centroid of the blue points
        print(count)
        centroid = np.mean(blue_points_xyz, axis=0)
        print("Centroid of blue points:", centroid)



    def process_coordinates(self, x, y, z):
        # Print the 3D coordinates in the terminal
        self.get_logger().info(f'3D Coordinates: X={x}, Y={y}, Z={z}')

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
