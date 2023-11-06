#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10)
        self.depth_image = None
        self.ball_pub = self.create_publisher(PoseStamped,'ball_pose',10)

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='16UC1')

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([(200/2), (80/100)*255, (30/100)*255])
        upper_blue = np.array([(240/2), (100/100)*255, (100/100)*255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # Mark the centroid on the mask
                cv2.circle(mask, (cX, cY), 5, (0, 0, 255), -1)

                if self.depth_image is not None:
                    # Mark the centroid on the depth image
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
                    cv2.circle(depth_colormap, (cX, cY), 5, (0, 0, 255), -1)

                    # Show the depth image
                    cv2.imshow('Depth Image with Centroid', depth_colormap)
                    cv2.waitKey(1)

                    depth = self.depth_image[cY, cX]
                    self.get_logger().info(f'Depth to centroid: {depth} units')

                    # Camera's field of view parameters
                    fov_horizontal = 83  # in degrees
                    fov_vertical = 53  # in degrees
                    
                    # Screen dimensions
                    height, width = self.depth_image.shape
                    
                    # Calculate yaw and pitch based on the centroid's position
                    yaw = ((cX - width / 2) / (width / 2)) * (fov_horizontal / 2)
                    pitch = -((cY - height / 2) / (height / 2)) * (fov_vertical / 2)

                    # Convert yaw and pitch to radians
                    yaw_rad = np.deg2rad(yaw)
                    pitch_rad = np.deg2rad(pitch)

                    # Calculate the 3D coordinates
                    y_coord = depth * np.cos(pitch_rad) * np.cos(yaw_rad) * 0.001
                    x_coord = depth * np.cos(pitch_rad) * np.sin(yaw_rad) * 0.001
                    z_coord = depth * np.sin(pitch_rad) * 0.001
                    
                    # Logging the 3D coordinates
                    self.get_logger().info(f'3D Coordinates: X={x_coord}, Y={y_coord}, Z={z_coord}')
                    pose = PoseStamped()
                    pose = PoseStamped()
                    pose.header.frame_id = "ball_pose"
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = x_coord
                    pose.pose.position.y = y_coord
                    pose.pose.position.z = z_coord
                    self.ball_pub.publish(pose)
                
        # Show the mask with centroid
        cv2.imshow('Mask with Centroid', mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
