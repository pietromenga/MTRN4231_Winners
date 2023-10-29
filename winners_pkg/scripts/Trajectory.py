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

        # Prediction publisher
        self.pred_publisher_ = self.create_publisher(PoseStamped, 'ball_prediction', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.dt = 0.5
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.last_x_hat = self.last_y_hat = self.last_z_hat = None
        self.vx_hat = self.vy_hat = self.vz_hat = 0

        self.alpha = 0.8  # Tunable parameter for position update
        self.beta = 0.02  # Tunable parameter for velocity update

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
        if self.last_x_hat is None:
            self.last_x_hat, self.last_y_hat, self.last_z_hat = x, y, z
            return

        # Alpha-Beta filter update
        self.vx_hat = self.vx_hat + self.beta * ((x - self.last_x_hat) - self.vx_hat * self.dt)
        self.vy_hat = self.vy_hat + self.beta * ((y - self.last_y_hat) - self.vy_hat * self.dt)
        self.vz_hat = self.vz_hat + self.beta * ((z - self.last_z_hat) - self.vz_hat * self.dt - 9.81 * 0.5 * self.dt**2)

        self.last_x_hat = self.last_x_hat + self.vx_hat * self.dt + self.alpha * ((x - self.last_x_hat) - self.vx_hat * self.dt)
        self.last_y_hat = self.last_y_hat + self.vy_hat * self.dt + self.alpha * ((y - self.last_y_hat) - self.vy_hat * self.dt)
        self.last_z_hat = self.last_z_hat + self.vz_hat * self.dt + self.alpha * ((z - self.last_z_hat) - self.vz_hat * self.dt - 9.81 * 0.5 * self.dt**2)

        # Predict next position
        predicted_x = self.last_x_hat + self.vx_hat * self.dt
        predicted_y = self.last_y_hat + self.vy_hat * self.dt
        predicted_z = self.last_z_hat + self.vz_hat * self.dt - 9.81 * 0.5 * self.dt**2  # accounting for gravity

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
