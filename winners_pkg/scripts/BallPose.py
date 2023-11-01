#!/usr/bin/env python3
# Importing necessary libraries
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BallPose(Node):

    def __init__(self):
        super().__init__('BallPose')

        # Define the positions of the cameras
        self.camera_position = {0: [10, 0, 0], 2: [-10, 0, 0]}

        # Ball pose publisher
        self.ball_pub = self.create_publisher(PoseStamped, 'ball_pose', 10)

        self.camSub0 = self.create_subscription(Image, '/camera0/image_raw', self.getYawPitch0, 10)
        self.camSub1 = self.create_subscription(Image, '/camera1/image_raw', self.getYawPitch1, 10)

        self.start_time = time.time()
        
        self.kernel = np.ones((5, 5), np.uint8)
        self.yawPitch0 = (0,0)
        self.yawPitch1 = (0,0)

        # BLUE
        # self.lower_range = np.array([105, 100, 100])
        # self.upper_range = np.array([125, 255, 255])
        # GREEN
        self.lower_range = np.array([60,25,25])
        self.upper_range = np.array([80,255,255])

    def getYawPitch0(self, imgMsg: Image):
        self.yawPitch0 = self.getYawPitch(imgMsg)
        # self.process_camera()

    def getYawPitch1(self, imgMsg: Image):
        self.yawPitch1 = self.getYawPitch(imgMsg)
        self.process_camera()

    def getYawPitch(self, imgMsg: Image):
        # Convert to cv image
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(imgMsg, desired_encoding='passthrough')

        # Convert colour
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Colour mask ball
        mask = cv2.inRange(hsv, self.lower_range, self.upper_range)

        # Clean up mask
        mask = cv2.erode(mask, self.kernel, iterations=3)
        mask = cv2.dilate(mask, self.kernel, iterations=3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # # Find centroid
        dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
        dist_output = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)
        indexMax = np.argmax(dist_output)
        y = int(np.floor(indexMax / dist.shape[1]))
        x = int(indexMax % dist.shape[1])

        height, width = img.shape[:2]
        # self.get_logger().info(f"{height}, {width}")
        fov_horizontal = 81.6  # in degrees
        fov_vertical = 54.4  # in degrees
        # Calculate yaw and pitch based on the blue dot's position
        yaw = ((x - width / 2) / (width / 2)) * (fov_horizontal / 2)
        pitch = -((y - height / 2) / (height / 2)) * (fov_vertical / 2)

        return yaw, pitch

    # Function to calculate the direction the camera is pointing
    def calculate_direction_vector(self, yaw, pitch):
        # Convert angles to radians
        yaw_rad = np.radians(yaw)
        pitch_rad = np.radians(pitch)
        # Calculate direction
        dx = np.sin(yaw_rad) * np.cos(pitch_rad)
        dy = np.cos(yaw_rad) * np.cos(pitch_rad)
        dz = np.sin(pitch_rad)
        return np.array([dx, dy, dz])

    # Function to find the closest point between two lines
    def closest_point_of_approach(self, p1, d1, p2, d2):
        # Math stuff to find the closest points
        w0 = p1 - p2
        a = np.dot(d1, d1)
        b = np.dot(d1, d2)
        c = np.dot(d2, d2)
        d = np.dot(d1, w0)
        e = np.dot(d2, w0)
        denom = a * c - b * b
        # Check if we can proceed
        if denom != 0:
            sc = (b * e - c * d) / denom
            tc = (a * e - b * d) / denom
            point_on_line1 = p1 + sc * d1
            point_on_line2 = p2 + tc * d2
        else:
            print("Denominator is zero. Cannot proceed with the calculation.")
            point_on_line1 = np.array([np.nan, np.nan, np.nan])
            point_on_line2 = np.array([np.nan, np.nan, np.nan])
        # Find the midpoint between the closest points
        midpoint = (point_on_line1 + point_on_line2) / 2
        return point_on_line1, point_on_line2, midpoint

    # Function to handle each camera
    def process_camera(self):
        try:
            # Extract yaw and pitch for both cameras
            yaw1, pitch1 = self.yawPitch0
            yaw2, pitch2 = self.yawPitch1
        except IndexError:
            # If the list is shorter than expected, print an error and continue
            print("IndexError caught. Shared list is shorter than expected.")
            return
        # Calculate direction vectors for both cameras
        d1 = self.calculate_direction_vector(yaw1, pitch1)
        d2 = self.calculate_direction_vector(yaw2, pitch2)
        # Get the camera positions
        p1 = np.array(self.camera_position[0]) #todo change
        p2 = np.array(self.camera_position[2])
        # Calculate the closest points and midpoint
        _, _, midpoint = self.closest_point_of_approach(p1, d1, p2, d2)
        if midpoint[1] < 0:
            midpoint *= -1.0

        if not np.isnan(midpoint).all():
            print(f"Midpoint between closest points: {midpoint}")

            midpoint *= 0.01

            pose = PoseStamped()
            pose.header.frame_id = "ball_pose"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = midpoint[0]
            pose.pose.position.y = midpoint[1]
            pose.pose.position.z = midpoint[2]
            self.ball_pub.publish(pose)

# Main function to set up the cameras
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = BallPose()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

# Run the main function when the script is executed
if __name__ == "__main__":
    main()