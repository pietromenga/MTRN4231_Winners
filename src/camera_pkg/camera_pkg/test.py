import rclpy
from rclpy.node import Node
import cv2


class MyNode(Node):
    def __init__(self):
        # Initialize the Video
        cap = cv2.VideoCapture('Videos/vid (4).mp4')

        # cv2.imshow("Image", img)
        cv2.imshow("ImageColor", cap)
        cv2.waitKey(100)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
