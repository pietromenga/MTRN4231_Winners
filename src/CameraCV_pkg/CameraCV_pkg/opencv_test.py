#!/usr/bin/env python3
# Importing necessary libraries
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from multiprocessing import Process, Manager, Lock
from scipy.spatial.distance import cdist

# Function to calculate the direction the camera is pointing
def calculate_direction_vector(yaw, pitch):
    # Convert angles to radians
    yaw_rad = np.radians(yaw)
    pitch_rad = np.radians(pitch)
    # Calculate direction
    dx = np.sin(yaw_rad) * np.cos(pitch_rad)
    dy = np.cos(yaw_rad) * np.cos(pitch_rad)
    dz = np.sin(pitch_rad)
    return np.array([dx, dy, dz])

# Function to find the closest point between two lines
def closest_point_of_approach(p1, d1, p2, d2):
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
def process_camera(camera_id, output_file, camera_position, shared_list, lock):
    # Initialize ROS node
    rclpy.init()
    node = Node("my_node_" + str(camera_id))
    # Set up the camera
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 60)
    # Set up video writer
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    out = cv2.VideoWriter(output_file, fourcc, 60, (1920, 1080))
    # Initialize variables for FPS calculation
    frame_count = 0
    start_time = time.time()
    fps = 0
    # Main loop to capture frames
    while True:
        # Capture a frame
        success, img = cap.read()
        # If we can't read a frame, warn and break
        if not success:
            node.get_logger().warn(f"Failed to read frame from camera {camera_id}")
            break
        # If we got a frame, then process it
        if img is not None:
            # Convert image to HSV color space
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # Define the color range for detecting blue
            lower_blue = np.array([100, 50, 50])
            upper_blue = np.array([140, 255, 255])
            # Create a mask to isolate blue regions
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            # Define a kernel for morphological operations
            kernel = np.ones((5, 5), np.uint8)
            # Clean up the mask
            mask_blue = cv2.erode(mask_blue, kernel, iterations=3)
            mask_blue = cv2.dilate(mask_blue, kernel, iterations=3)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
            # Find the blue dot in the mask
            dist = cv2.distanceTransform(mask_blue, cv2.DIST_L2, 5)
            dist_output = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)
            # Get the coordinates of the blue dot
            indexMax = np.argmax(dist_output)
            y = int(np.floor(indexMax / dist.shape[1]))
            x = int(indexMax % dist.shape[1])
            # Draw a blue circle around the blue dot
            cv2.circle(img, (x, y), 10, (255, 0, 0), -1)
            # Calculate the camera's field of view
            height, width = img.shape[:2]
            fov_horizontal = 83  # in degrees
            fov_vertical = 53  # in degrees
            # Calculate yaw and pitch based on the blue dot's position
            yaw = ((x - width / 2) / (width / 2)) * (fov_horizontal / 2)
            pitch = -((y - height / 2) / (height / 2)) * (fov_vertical / 2)
            # Store yaw and pitch in the shared list
            shared_list.append((camera_id, yaw, pitch))
            # Lock the shared list before accessing
            with lock:
                # Check if both cameras have stored their yaw and pitch
                if len(shared_list) >= 2:
                    try:
                        # Extract yaw and pitch for both cameras
                        id1, yaw1, pitch1 = shared_list[0]
                        id2, yaw2, pitch2 = shared_list[1]
                    except IndexError:
                        # If the list is shorter than expected, print an error and continue
                        print("IndexError caught. Shared list is shorter than expected.")
                        continue
                    # Calculate direction vectors for both cameras
                    d1 = calculate_direction_vector(yaw1, pitch1)
                    d2 = calculate_direction_vector(yaw2, pitch2)
                    # Get the camera positions
                    p1 = np.array(camera_position[id1])
                    p2 = np.array(camera_position[id2])
                    # Calculate the closest points and midpoint
                    point1, point2, midpoint = closest_point_of_approach(p1, d1, p2, d2)
                    print(f"Midpoint between closest points: {midpoint}")
                    # Clear the shared list for the next iteration
                    shared_list[:] = []
            # Show the original image
            cv2.imshow(f"Camera {camera_id} Stream", img)
            # Write the frame to the output video
            out.write(img)
            # Update the frame count
            frame_count += 1
            # Calculate the elapsed time
            elapsed_time = time.time() - start_time
            # Update the FPS every second
            if elapsed_time > 1:
                fps = frame_count / elapsed_time
                print(f"Camera {camera_id} FPS: {fps:.2f}")
                frame_count = 0
                start_time = time.time()
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    # Release the video writer and destroy all OpenCV windows
    out.release()
    cv2.destroyAllWindows()
    # Shutdown the ROS node
    rclpy.shutdown()

# Main function to set up the cameras
def main(args=None):
    # Create a lock for the shared list
    lock = Lock()
    # Print the OpenCV version
    print(cv2.__version__)
    # Create a manager for the shared list
    manager = Manager()
    # Create the shared list
    shared_list = manager.list()
    # Define the positions of the cameras
    camera_position = {0: [10, 0, 0], 2: [-10, 0, 0]}
    # Create processes for each camera
    p1 = Process(target=process_camera, args=(0, "compressed_output0.avi", camera_position, shared_list, lock))
    p2 = Process(target=process_camera, args=(2, "compressed_output1.avi", camera_position, shared_list, lock))
    # Start the camera processes
    p1.start()
    p2.start()
    # Wait for both processes to finish
    p1.join()
    p2.join()

# Run the main function when the script is executed
if __name__ == "__main__":
    main()
