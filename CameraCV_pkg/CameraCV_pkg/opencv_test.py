#!/usr/bin/env python3
# Importing necessary libraries
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from multiprocessing import Process, Manager, Lock
from scipy.spatial.distance import cdist
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time 

camera1_id = 0
camera2_id = 2

class BallPose(Node):

    def __init__(self):
        super().__init__('Trajectory_Calculator')

        # Create a lock for the shared list
        self.lock = Lock()
        # Print the OpenCV version
        print(cv2.__version__)
        # Create a manager for the shared list
        self.manager = Manager()
        # Create the shared list
        self.shared_list = self.manager.list()
        # Define the positions of the cameras
        self.camera_position = {0: [10, 0, 0], 2: [-10, 0, 0]}
        # Create processes for each camera

        self.ball_pub = self.create_publisher(PoseStamped, 'ball_pose', 10)
    
        # Set up the camera
        self.id1 = camera1_id
        self.id2 = camera2_id
        self.cap1 = cv2.VideoCapture(self.id1)
        self.cap2 = cv2.VideoCapture(self.id2)
        self.caps = [self.cap1, self.cap2]

        for cap in self.caps:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            cap.set(cv2.CAP_PROP_FPS, 60)

        # Set up video writer
        # fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        # out = cv2.VideoWriter(output_file, fourcc, 60, (1920, 1080))
        # Initialize variables for FPS calculation
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0
        
        self.timer1 = self.create_timer(0.01, self.process_camera1)
        self.timer2 = self.create_timer(0.01, self.process_camera2)

        self.kernel = np.ones((5, 5), np.uint8)
        # BLUE
        # self.lower_range = np.array([105, 100, 100])
        # self.upper_range = np.array([125, 255, 255])
        # GREEN
        self.lower_range = np.array([60,25,25])
        self.upper_range = np.array([80,255,255])

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

    def process_camera1(self):
        self.process_camera(0)

    def process_camera2(self):
        self.process_camera(1)

    # Function to handle each camera
    def process_camera(self, camera_id):
        # Capture a frame
        success, img = self.caps[camera_id].read()
        # If we can't read a frame, warn and break
        if not success:
            self.get_logger().warn(f"Failed to read frame from camera {camera_id}")
            rclpy.shutdown()

        # If we got a frame, then process it
        if img is not None:
            # Convert image to HSV color space
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # Define the color range for detecting blue

            # Define the color range for detecting blue
            # lower_blue = np.array([100, 50, 50])
            # upper_blue = np.array([140, 255, 255])
            
            # Create a mask to isolate blue regions
            mask = cv2.inRange(hsv, self.lower_range, self.upper_range)

            # Define a kernel for morphological operations
            # Clean up the mask
            mask = cv2.erode(mask, self.kernel, iterations=3)
            mask = cv2.dilate(mask, self.kernel, iterations=3)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            # Find the blue dot in the mask
            dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
            dist_output = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)
            # Get the coordinates of the blue dot
            indexMax = np.argmax(dist_output)
            y = int(np.floor(indexMax / dist.shape[1]))
            x = int(indexMax % dist.shape[1])
            # Draw a blue circle around the blue dot
            cv2.circle(img, (x, y), 10, (255, 0, 0), -1)
            # Calculate the camera's field of view
            height, width = img.shape[:2]
            fov_horizontal = 81.6  # in degrees
            fov_vertical = 54.4  # in degrees
            # Calculate yaw and pitch based on the blue dot's position
            yaw = ((x - width / 2) / (width / 2)) * (fov_horizontal / 2)
            pitch = -((y - height / 2) / (height / 2)) * (fov_vertical / 2)
            # Lock the shared list before accessing
            with self.lock:
                # Store yaw and pitch in the shared list
                self.shared_list.append((camera_id, yaw, pitch))
                # Check if both cameras have stored their yaw and pitch
                if len(self.shared_list) >= 2:
                    try:
                        # Extract yaw and pitch for both cameras
                        id1, yaw1, pitch1 = self.shared_list[0]
                        id2, yaw2, pitch2 = self.shared_list[1]
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
                    point1, point2, midpoint = self.closest_point_of_approach(p1, d1, p2, d2)
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

                    # Clear the shared list for the next iteration
                    self.shared_list[:] = []

            # Show the original image
            cv2.imshow(f"Camera {camera_id} Stream", mask)
            # Write the frame to the output video
            # out.write(img)
            # Update the frame count
            self.frame_count += 1
            # Calculate the elapsed time
            elapsed_time = time.time() - self.start_time
            # Update the FPS every second
            if elapsed_time > 1:
                fps = self.frame_count / elapsed_time
                print(f"Camera {camera_id} FPS: {fps:.2f}")
                self.frame_count = 0
                self.start_time = time.time()
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rclpy.shutdown

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