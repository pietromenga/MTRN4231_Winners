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
        super().__init__("ball_detector_node")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10
        )
        self.depth_subscription = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.depth_callback, 10
        )
        self.depth_image = None
        self.ball_pub = self.create_publisher(PoseStamped, "ball_pose", 10)
        # BLUE
        # self.lower_range = np.array([(200/2), (80/100)*255, (30/100)*255])
        # self.upper_blue = np.array([(240/2), (100/100)*255, (100/100)*255])
        # BALL GREEN
        self.lower_range = np.array([137.5 / 2, 150, 0.05 * 255])
        self.upper_range = np.array([155.8 / 2, 255, 0.8 * 255])

        self.kernel = np.ones((3, 3), np.uint8)

        self.fov_horizontal = 69.4  # in degrees
        self.fov_vertical = 42.5  # in degrees

        self.debug = False

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="16UC1")

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Mask for balls
        mask_balls = cv2.inRange(hsv, self.lower_range, self.upper_range)
        mask_balls = cv2.morphologyEx(mask_balls, cv2.MORPH_OPEN, self.kernel)
        contours_balls, _ = cv2.findContours(
            mask_balls, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        # Mask for glove
        mask_glove = cv2.inRange(hsv, self.lower_glove_range, self.upper_glove_range)
        mask_glove = cv2.morphologyEx(mask_glove, cv2.MORPH_OPEN, self.kernel)
        contours_glove, _ = cv2.findContours(
            mask_glove, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        ball_centroids = []
        glove_centroid = None

        # Find centroids of all balls
        for contour in contours_balls:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                ball_centroids.append((cX, cY))

        # Find centroid of glove
        if contours_glove:
            largest_contour_glove = max(contours_glove, key=cv2.contourArea)
            M = cv2.moments(largest_contour_glove)
            if M["m00"] != 0:
                glove_centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # Determine closest ball to glove
        if glove_centroid and ball_centroids:
            closest_ball = min(
                ball_centroids,
                key=lambda centroid: np.linalg.norm(
                    np.array(centroid) - np.array(glove_centroid)
                ),
            )

            if self.depth_image is not None:
                cX, cY = closest_ball
                depth = self.depth_image[cY, cX]

                # Screen dimensions
                height, width = self.depth_image.shape

                # Calculate yaw and pitch based on the centroid's position
                yaw = ((cX - width / 2) / (width / 2)) * (self.fov_horizontal / 2)
                pitch = -((cY - height / 2) / (height / 2)) * (self.fov_vertical / 2)

                # Convert yaw and pitch to radians
                yaw_rad = np.deg2rad(yaw)
                pitch_rad = np.deg2rad(pitch)

                # Calculate the 3D coordinates
                y_coord = depth * np.cos(pitch_rad) * np.cos(yaw_rad) * 0.001
                x_coord = depth * np.cos(pitch_rad) * np.sin(yaw_rad) * 0.001
                z_coord = depth * np.sin(pitch_rad) * 0.001

                # Logging the 3D coordinates
                # self.get_logger().info(f'3D Coordinates: X={x_coord}, Y={y_coord}, Z={z_coord}')
                pose = PoseStamped()
                pose.header.frame_id = "ball_pose"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = x_coord
                pose.pose.position.y = y_coord
                pose.pose.position.z = z_coord
                self.ball_pub.publish(pose)

        # Debugging output
        if self.debug:
            # Display the masks and centroids
            for cX, cY in ball_centroids:
                cv2.circle(mask_balls, (cX, cY), 5, (0, 0, 255), -1)
            if glove_centroid:
                cv2.circle(mask_glove, glove_centroid, 5, (255, 0, 0), -1)
            cv2.imshow("Balls Mask", mask_balls)
            cv2.imshow("Glove Mask", mask_glove)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
