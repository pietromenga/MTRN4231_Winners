#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from collections import deque
from TriKalman import KalmanFilter3D
import numpy as np

class TrajectoryCalculator(Node):

    def __init__(self):
        super().__init__('Trajectory_Calculator')

        self.pred_publisher_ = self.create_publisher(PoseStamped, 'ball_prediction', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initial state estimate [x, y, z, vx, vy, vz]
        initial_state = np.array([0, 0, 0, 1, 1, 1])

        # Initial covariance matrix
        initial_covariance = np.eye(6)

        # Process noise covariance matrix
        process_noise = np.eye(6) * 0.01

        # Measurement noise covariance matrix
        measurement_noise = np.eye(6) * 0.1

        # Create a Kalman filter instance
        self.kf = KalmanFilter3D(initial_state, initial_covariance, process_noise, measurement_noise)

        self.dt = 1.0  # Prediction time
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.last_x = self.last_y = self.last_z = None
        self.last_time = None

        self.last_positions = deque(maxlen=5)  # Stores the last 5 positions
        self.last_times = deque(maxlen=5)  # Stores the last 5 timestamps

    def timer_callback(self):

        try:
            tBall = self.tf_buffer.lookup_transform('base_link', 'ball_tf', rclpy.time.Time())
        except TransformException as ex:
            return

        current_time = tBall.header.stamp.sec + tBall.header.stamp.nanosec * 1e-9  # Convert to seconds
        current_position = (tBall.transform.translation.x, tBall.transform.translation.y, tBall.transform.translation.z)

        self.last_positions.append(current_position)
        self.last_times.append(current_time)

        if len(self.last_positions) < 5:
            return  # Not enough data points yet

        # Calculate average velocity
        dx = dy = dz = dt = 0
        for i in range(4):
            dx += self.last_positions[i+1][0] - self.last_positions[i][0]
            dy += self.last_positions[i+1][1] - self.last_positions[i][1]
            dz += self.last_positions[i+1][2] - self.last_positions[i][2]
            dt += self.last_times[i+1] - self.last_times[i]

        # Safety measure to avoid division by zero
        if dt == 0:
            return

        vx = dx / dt
        vy = dy / dt
        vz = dz / dt

        # Predict next position
        measurements = np.array([current_position[0], current_position[1], current_position[2], vx, vy, vz])
        self.kf.predict()
        self.kf.update(measurements)

        self.sendBallPred(self.kf.state[0], self.kf.state[1], self.kf.state[2])

        # This line is the culprit. Replace x, y, z with current_position[0], current_position[1], current_position[2]
        self.last_x, self.last_y, self.last_z = current_position[0], current_position[1], current_position[2]
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