#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
import time
from builtin_interfaces.msg import Time

from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TrajectoryCalculator(Node):
    def __init__(self):
        super().__init__("Trajectory_Calculator")

        # Prediction publisher
        self.pred_publisher_ = self.create_publisher(PoseStamped, "ball_prediction", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.dt = 0.1
        # self.timer = self.create_timer(self.dt, self.timer_callback)
        self.create_subscription(PoseStamped, "/ball_pose", self.timer_callback, 10)

        self.itemCount = 0
        self.posX = []
        self.posY = []
        self.posZ = []
        self.timeList = []
        self.startTime = time.time()
        self.to_frame_rel = "base_link"

        self.minY, self.maxY = 0, 0.4
        self.minX, self.maxX = -0.9, -0.5
        self.minZ, self.maxZ = 0, 0.25

    def timer_callback(self, pose: PoseStamped):
        # Get position of ball in base frame
        # try:
        #     tBall = self.tf_buffer.lookup_transform(
        #         self.to_frame_rel, "ball_tf", rclpy.time.Time()
        #     )
        # except TransformException as ex:
        #     return

        self.appendTransform(pose)

        # Find target
        ballPredTarget = self.predictBall()

        # send off prediction
        if ballPredTarget != ():
            self.sendBallPred(*ballPredTarget)  # unpack tuple into args

    def predictBall(self):
        ballPredTarget = ()

        # Ensure we have at least two points to calculate velocity
        if self.itemCount >= 2:
            # Calculate velocities
            vx = (self.posX[-1] - self.posX[-2]) / (
                self.timeList[-1] - self.timeList[-2]
            )
            vy = (self.posY[-1] - self.posY[-2]) / (
                self.timeList[-1] - self.timeList[-2]
            )
            vz = (self.posZ[-1] - self.posZ[-2]) / (
                self.timeList[-1] - self.timeList[-2]
            )

            # Gravity constant in m/s^2 (negative because it's acting downwards)
            gravity = -9.81

            # z = -0.2 plane
            z_target = -0.2

            # Calculate time to reach z = -0.2 plane using quadratic formula
            A = 0.5 * gravity
            B = vz
            C = self.posZ[-1] - z_target

            # Calculate discriminant
            discriminant = B**2 - 4 * A * C
            if discriminant >= 0:
                # Compute the two possible solutions for time
                t1 = (-B + np.sqrt(discriminant)) / (2 * A)
                t2 = (-B - np.sqrt(discriminant)) / (2 * A)
                # We want the positive, smallest time at which the ball crosses z = -0.2
                t = min(filter(lambda t: t > 0, [t1, t2]), default=None)

                if t is not None:
                    # Calculate the z-velocity at the intersection time
                    vz_intersection = vz + gravity * t

                    # Ensure that the z-velocity is negative (ball is moving downwards)
                    if vz_intersection < 0:
                        # Calculate future positions for x and y
                        pred_x = self.posX[-1] + vx * t
                        pred_y = self.posY[-1] + vy * t

                        # We already know pred_z since it's the z_target
                        pred_z = z_target

                        ballPredTarget = (pred_x, pred_y, pred_z)

        return ballPredTarget

    def sendBallPred(self, x, y, z):
        pred = PoseStamped()
        pred.header.stamp = self.get_clock().now().to_msg()
        pred.header.frame_id = "base_link"
        pred.pose.position.x = float(x)
        pred.pose.position.y = float(y)
        pred.pose.position.z = float(z)
        self.pred_publisher_.publish(pred)

    def inCatchingRange(self, x, y, z):
        # xRange = x > self.minX and x < self.maxX
        # yRange = y > self.minY and y < self.maxY
        # zRange = z > self.minZ and z < self.maxZ
        return True

    def clearLists(self):
        self.posX.clear()
        self.posY.clear()
        self.posZ.clear()
        self.timeList.clear()
        self.itemCount = 0

    def appendTransform(self, pose: PoseStamped):
        self.posX.append(pose.pose.position.x)
        self.posY.append(pose.pose.position.y)
        self.posZ.append(pose.pose.position.z)
        self.timeList.append(time.time() - self.startTime)
        self.itemCount += 1
        # self.get_logger().info(f"Time: {time.time() - self.startTime}")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = TrajectoryCalculator()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
