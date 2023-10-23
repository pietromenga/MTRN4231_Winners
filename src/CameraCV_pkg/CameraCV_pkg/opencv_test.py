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

    # Initialize blob detector
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 10  # Adjust this value based on your specific use case
    detector = cv2.SimpleBlobDetector_create(params)

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
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = (160, 120, 70)
        upper_red = (180, 255, 255)
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2
        ######### PIETRO CHANGES #############

        # GREEN MASK
        mask = cv2.inRange(hsv, (36,25,25), (76,240,240))

        # Processing mask to reduce noise
        mask = cv2.erode(mask, kernel, iterations=3)
        mask = cv2.dilate(mask, kernel, iterations=3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # METHOD 1 - DISTANCE
        dist = cv2.distanceTransform(mask,cv2.DIST_L2, 5)
        dist_output = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX) 
        indexMax = np.argmax(dist_output)
        y = int(np.floor(indexMax / dist.shape[1]))
        x = int(indexMax % dist.shape[1])
        cv2.circle(img, (x, y), 10, (255, 0, 0), -1)  # Green circle

        # METHOD 2 - https://pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        # cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cnts = imutils.grab_contours(cnts)
        # if len(cnts) > 0:
        #     # find the largest contour in the mask, then use
        #     # it to compute the minimum enclosing circle and
        #     # centroid
        #     c = max(cnts, key=cv2.contourArea)
        #     ((x, y), radius) = cv2.minEnclosingCircle(c)
        #     M = cv2.moments(c)
        #     center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        #     cv2.circle(img, center, 10, (255, 0, 0), -1)

        #####################################

        # # Detect blobs in the mask
        # keypoints = detector.detect(mask)

        # # Draw detected blobs on the original image
        # for keypoint in keypoints:
        #     x, y = map(int, keypoint.pt)
        #     cv2.circle(img, (x, y), 7, (0, 255, 0), -1)  # Green circle

        # Add FPS counter to the original image
        # cv2.putText(img, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

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
    p1 = Process(target=process_camera, args=(0, 'compressed_output0.avi'))
    p2 = Process(target=process_camera, args=(2, 'compressed_output1.avi'))

    p1.start()
    p2.start()

    p1.join()
    p2.join()

if __name__ == '__main__':
    main()
