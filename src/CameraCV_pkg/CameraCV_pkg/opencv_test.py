#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from multiprocessing import Process

def process_camera(camera_id, output_file):
    rclpy.init()
    node = Node('my_node_' + str(camera_id))

    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 60)

    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
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

            print(f"Camera {camera_id} Blue Dot Coordinates: X: {x}, Y: {y}")

            height, width = img.shape[:2]
            fov_horizontal = 90  # horizontal field of view in degrees
            fov_vertical = 50.625  # vertical field of view in degrees

            yaw = ((x - width / 2) / (width / 2)) * (fov_horizontal / 2)
            pitch = -((y - height / 2) / (height / 2)) * (fov_vertical / 2)

            print(f"Camera {camera_id} Yaw: {yaw:.2f}, Pitch: {pitch:.2f}")

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

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    out.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

def main(args=None):
    print(cv2.__version__)

    p1 = Process(target=process_camera, args=(0, 'compressed_output0.avi'))
    p2 = Process(target=process_camera, args=(2, 'compressed_output1.avi'))

    p1.start()
    p2.start()

    p1.join()
    p2.join()

if __name__ == '__main__':
    main()
