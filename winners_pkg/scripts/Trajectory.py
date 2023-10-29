#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TrajectoryCalculator(Node):

    def __init__(self):
        super().__init__('Trajectory_Calculator')

        self.pred_publisher_ = self.create_publisher(PoseStamped, 'ball_prediction', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.dt = 0.5  # Prediction time
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.last_x = self.last_y = self.last_z = None
        self.last_time = None

    def timer_callback(self):

        try:
            tBall = self.tf_buffer.lookup_transform('base_link', 'ball_tf', rclpy.time.Time())
        except TransformException as ex:
            return

        current_time = tBall.header.stamp.sec + tBall.header.stamp.nanosec * 1e-9  # Convert to seconds
        x, y, z = tBall.transform.translation.x, tBall.transform.translation.y, tBall.transform.translation.z

        if self.last_x is not None:
            elapsed_time = current_time - self.last_time
            vx = (x - self.last_x) / elapsed_time
            vy = (y - self.last_y) / elapsed_time
            vz = (z - self.last_z) / elapsed_time

            # Predict next position
            predicted_x = x + vx * self.dt
            predicted_y = y + vy * self.dt
            predicted_z = z + vz * self.dt

            self.sendBallPred(predicted_x, predicted_y, predicted_z)

        self.last_x, self.last_y, self.last_z = x, y, z
        self.last_time = current_time

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
