#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import time

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.cap1 = cv2.VideoCapture(0)
        self.cap2 = cv2.VideoCapture(2)

        # Set resolution to 1080p for both cameras
        for cap in [self.cap1, self.cap2]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            cap.set(cv2.CAP_PROP_FPS, 60)

        self.show_video()

    def show_video(self):
        frame_count = 0
        start_time = time.time()
        fps = 0  # Initialize fps to 0

        while True:
            success1, img1 = self.cap1.read()
            success2, img2 = self.cap2.read()

            if not success1:
                self.get_logger().warn("Failed to read frame from camera 1")
                break

            if not success2:
                self.get_logger().warn("Failed to read frame from camera 2")
                break

            frame_count += 1
            elapsed_time = time.time() - start_time

            if elapsed_time > 1:  # every second
                fps = frame_count / elapsed_time
                print(f"FPS: {fps:.2f}")
                frame_count = 0
                start_time = time.time()

            cv2.putText(img1, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(img2, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Camera 1 Stream", img1)
            cv2.imshow("Camera 2 Stream", img2)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
