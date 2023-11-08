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
        if self.itemCount >= 10:
            xCoeff = np.polyfit(self.timeList, self.posX, 2)
            yCoeff = np.polyfit(self.timeList, self.posY, 2)
            zCoeff = np.polyfit(self.timeList, self.posZ, 2)

            ti = time.time() - self.startTime + 1.0
            x = float(xCoeff[0] * ti ** 2 + xCoeff[1] * ti + xCoeff[2])
            y = float(yCoeff[0] * ti ** 2 + yCoeff[1] * ti + yCoeff[2])
            z = float(zCoeff[0] * ti ** 2 + zCoeff[1] * ti + zCoeff[2])

            # # Calculate velocities
            # vx = (self.posX[-1] - self.posX[-2]) / (
            #     self.timeList[-1] - self.timeList[-2]
            # )
            # vy = (self.posY[-1] - self.posY[-2]) / (
            #     self.timeList[-1] - self.timeList[-2]
            # )
            # vz = (self.posZ[-1] - self.posZ[-2]) / (
            #     self.timeList[-1] - self.timeList[-2]
            # )

            # # Predict positions one second into the future
            # dt = 1.0 # One second into the future
            # gravity = -9.81  # Gravity constant in m/s^2

            # # Since x and y are not affected by gravity, we only apply the velocity
            # pred_x = self.posX[-1] + vx * dt
            # pred_y = self.posY[-1] + vy * dt

            # # For z, we apply gravity
            # pred_z = self.posZ[-1] + vz * dt + 0.5 * gravity * dt**2

            ballPredTarget = (x, y, z)
            self.clearLists()  # This might not be necessary; see the comment below

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
