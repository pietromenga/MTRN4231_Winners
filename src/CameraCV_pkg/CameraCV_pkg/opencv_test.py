#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
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

        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for red color
        lower_red = (0, 120, 70)
        upper_red = (10, 255, 255)

        # Create a mask for red color
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = (160, 120, 70)
        upper_red = (180, 255, 255)
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Show the mask
        cv2.imshow(f"Camera {camera_id} Red Mask", mask)

        out.write(img)
        frame_count += 1
        elapsed_time = time.time() - start_time

        if elapsed_time > 1:
            fps = frame_count / elapsed_time
            print(f"Camera {camera_id} FPS: {fps:.2f}")
            frame_count = 0
            start_time = time.time()

        cv2.putText(img, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow(f"Camera {camera_id} Stream", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    out.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    p1 = Process(target=process_camera, args=(0, 'compressed_output0.avi'))
    p2 = Process(target=process_camera, args=(2, 'compressed_output1.avi'))

    p1.start()
    p2.start()

    p1.join()
    p2.join()

def main(args=None):
    p1 = Process(target=process_camera, args=(0, 'compressed_output0.avi'))
    p2 = Process(target=process_camera, args=(2, 'compressed_output1.avi'))

    p1.start()
    p2.start()

    p1.join()
    p2.join()

if __name__ == '__main__':
    main()
