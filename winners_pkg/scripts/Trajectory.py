#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TrajectoryCalculator(Node):

    def __init__(self):
        super().__init__('Trajectory_Calculator')

        # Prediction publisher
        self.pred_publisher_ = self.create_publisher(PoseStamped, 'ball_prediction', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.last_x = self.last_y = self.last_z = None
        self.vx = self.vy = self.vz = 0

    def timer_callback(self):

        # Get position of ball in base frame
        try:
            tBall = self.tf_buffer.lookup_transform(
                'base_link',
                'ball_tf',
                rclpy.time.Time())
        except TransformException as ex:
            return

        # Current position
        x, y, z = tBall.transform.translation.x, tBall.transform.translation.y, tBall.transform.translation.z

        # If this is the first measurement, just store the position
        if self.last_x is None:
            self.last_x, self.last_y, self.last_z = x, y, z
            return

        # Compute velocity (assuming constant time step for simplicity)
        self.vx = (x - self.last_x) / self.dt
        self.vy = (y - self.last_y) / self.dt
        self.vz = (z - self.last_z) / self.dt

        # Predict next position
        predicted_x = x + self.vx * self.dt
        predicted_y = y + self.vy * self.dt
        predicted_z = z + self.vz * self.dt - 9.81 * 0.5 * self.dt**2  # accounting for gravity

        # Update last position
        self.last_x, self.last_y, self.last_z = x, y, z

        # Send off prediction
        self.sendBallPred(predicted_x, predicted_y, predicted_z)

    def sendBallPred(self, x, y, z):
        pred = PoseStamped()
        pred.header.stamp = self.get_clock().now().to_msg()
        pred.header.frame_id = "base_link"
        pred.pose.position.x = x
        pred.pose.position.y = y
        pred.pose.position.z = z
        self.pred_publisher_.publish(pred)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = TrajectoryCalculator()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
