#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from multiprocessing import Process
from multiprocessing import Manager
from multiprocessing import Lock
from scipy.spatial.distance import cdist


def calculate_direction_vector(yaw, pitch):
    yaw_rad = np.radians(yaw)
    pitch_rad = np.radians(pitch)
    dx = np.sin(yaw_rad) * np.cos(pitch_rad)
    dy = np.cos(yaw_rad) * np.cos(pitch_rad)
    dz = np.sin(pitch_rad)
    return np.array([dx, dy, dz])


def closest_point_of_approach(p1, d1, p2, d2):
    w0 = p1 - p2
    a = np.dot(d1, d1)
    b = np.dot(d1, d2)
    c = np.dot(d2, d2)
    d = np.dot(d1, w0)
    e = np.dot(d2, w0)
    denom = a * c - b * b

    # Check if denom is zero
    if denom != 0:
        sc = (b * e - c * d) / denom
        tc = (a * e - b * d) / denom
        point_on_line1 = p1 + sc * d1
        point_on_line2 = p2 + tc * d2
    else:
        print("Denominator is zero. Cannot proceed with the calculation.")
        point_on_line1 = np.array([np.nan, np.nan, np.nan])
        point_on_line2 = np.array([np.nan, np.nan, np.nan])

    # Calculate the midpoint between the closest points
    midpoint = (point_on_line1 + point_on_line2) / 2

    return point_on_line1, point_on_line2, midpoint


def process_camera(camera_id, output_file, camera_position, shared_list, lock):
    rclpy.init()
    node = Node("my_node_" + str(camera_id))

    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 60)

    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    out = cv2.VideoWriter(output_file, fourcc, 60, (1920, 1080))

    frame_count = 0
    start_time = time.time()
    fps = 0

    while True:
        success, img = cap.read()
        if not success:
            node.get_logger().warn(f"Failed to read frame from camera {camera_id}")
            break

        if img is not None:
            # Convert to HSV color space
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Define lower and upper bounds for blue color
            lower_blue = np.array([100, 50, 50])
            upper_blue = np.array([140, 255, 255])
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

            # Define the kernel
            kernel = np.ones((5, 5), np.uint8)

            # Processing mask to reduce noise
            mask_blue = cv2.erode(mask_blue, kernel, iterations=3)
            mask_blue = cv2.dilate(mask_blue, kernel, iterations=3)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)

            # Find the blue dot
            dist = cv2.distanceTransform(mask_blue, cv2.DIST_L2, 5)
            dist_output = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)
            indexMax = np.argmax(dist_output)
            y = int(np.floor(indexMax / dist.shape[1]))
            x = int(indexMax % dist.shape[1])
            cv2.circle(img, (x, y), 10, (255, 0, 0), -1)  # Blue circle

            # print(f"Camera {camera_id} Blue Dot Coordinates: X: {x}, Y: {y}")

            height, width = img.shape[:2]
            fov_horizontal = 83  # horizontal field of view in degrees
            fov_vertical = 53  # vertical field of view in degrees

            yaw = ((x - width / 2) / (width / 2)) * (fov_horizontal / 2)
            pitch = -((y - height / 2) / (height / 2)) * (fov_vertical / 2)

            # print(f"Camera {camera_id} Yaw: {yaw:.2f}, Pitch: {pitch:.2f}")

            # Store yaw and pitch in the shared list
            shared_list.append((camera_id, yaw, pitch))

            # Inside process_camera()
            with lock:  # Acquire lock before accessing shared_list
                if len(shared_list) >= 2:
                    try:
                        id1, yaw1, pitch1 = shared_list[0]
                        id2, yaw2, pitch2 = shared_list[1]
                    except IndexError:
                        print("IndexError caught. Shared list is shorter than expected.")
                        continue  # Skip the rest of the loop iteration

                    # Calculate direction vectors
                    d1 = calculate_direction_vector(yaw1, pitch1)
                    d2 = calculate_direction_vector(yaw2, pitch2)

                    # Camera positions
                    p1 = np.array(camera_position[id1])
                    p2 = np.array(camera_position[id2])

                    # Calculate the closest points and midpoint
                    point1, point2, midpoint = closest_point_of_approach(p1, d1, p2, d2)
                    print(f"Midpoint between closest points: {midpoint}")

                    # Clear the shared list for the next iteration
                    shared_list[:] = []

        # Show the original image
        cv2.imshow(f"Camera {camera_id} Stream", img)

        out.write(img)
        frame_count += 1
        elapsed_time = time.time() - start_time

        if elapsed_time > 1:
            fps = frame_count / elapsed_time
            print(f"Camera {camera_id} FPS: {fps:.2f}")
            frame_count = 0
            start_time = time.time()

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    out.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()


def main(args=None):

    lock = Lock()

    print(cv2.__version__)

    manager = Manager()
    shared_list = manager.list()

    camera_position = {0: [10, 0, 0], 2: [-10, 0, 0]}

    p1 = Process(
        target=process_camera,
        args=(0, "compressed_output0.avi", camera_position, shared_list, lock),
    )
    p2 = Process(
        target=process_camera,
        args=(2, "compressed_output1.avi", camera_position, shared_list, lock),
    )

    p1.start()
    p2.start()

    p1.join()
    p2.join()


if __name__ == "__main__":
    main()
