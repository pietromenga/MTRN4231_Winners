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
        super().__init__('Trajectory_Calculator')

        # Prediction publisher
        self.pred_publisher_ = self.create_publisher(PoseStamped, 'ball_prediction', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.itemCount = 0
        self.posX = []
        self.posY = []
        self.posZ = []
        self.timeList = []
        self.startTime = time.time()
        self.to_frame_rel = 'base_link'

        self.minY, self.maxY = 0, 0.4
        self.minX, self.maxX = -0.9, -0.5
        self.minZ, self.maxZ = 0, 0.25

    def timer_callback(self):

        # Get position of ball in base frame
        try:
            tBall = self.tf_buffer.lookup_transform(
                self.to_frame_rel,
                'ball_tf',
                rclpy.time.Time())
            
            tEndEff = self.tf_buffer.lookup_transform(
                self.to_frame_rel,
                'tool0',
                rclpy.time.Time())
        except TransformException as ex:
            # self.get_logger().info(
            #     f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # Store transforms
        if self.itemCount < 4:
            self.appendTransform(tBall)
        else: # Enough to regress
            currentTime = time.time() - self.startTime
            self.appendTransform(tBall)

            xCoeff = np.polyfit(self.timeList, self.posX, 2)
            yCoeff = np.polyfit(self.timeList, self.posY, 2)
            zCoeff = np.polyfit(self.timeList, self.posZ, 2)

            dt = 0.05
            timeSteps = np.arange(currentTime, currentTime + 3, dt)
            minDist = 100
            ballPredTarget = ()
            for ti in reversed(timeSteps): # Reversed as solutions farthest away will be closest
                # xyz at time step ti
                x = int(xCoeff[0] * ti ** 2 + xCoeff[1] * ti + xCoeff[2])
                y = int(yCoeff[0] * ti ** 2 + yCoeff[1] * ti + yCoeff[2])
                z = int(zCoeff[0] * ti ** 2 + zCoeff[1] * ti + zCoeff[2])

                # Calc distance to tool
                dist2tool = np.sqrt(
                    (x - tEndEff.transform.translation.x)**2 + 
                    (y - tEndEff.transform.translation.y)**2 +
                    (z - tEndEff.transform.translation.z)**2 
                )

                # Update mindist
                if dist2tool < minDist and self.inCatchingRange(x,y,z):
                    minDist = dist2tool 
                    ballPredTarget = (x,y,z)

            # send off prediction
            if ballPredTarget != ():
                self.sendBallPred(*ballPredTarget) #unpack tuple into args

            # Remove last point to get new regression
            self.removeLast()

    def sendBallPred(self, x,y,z):
        pred = PoseStamped()
        pred.header.stamp = self.get_clock().now()
        pred.header.frame_id = "base_link"
        pred.pose.position.x = x
        pred.pose.position.y = y
        pred.pose.position.z = z
        self.pred_publisher_.publish(pred)

    def inCatchingRange(self, x, y, z):
        xRange = x > self.minX and x < self.maxX
        yRange = y > self.minY and y < self.maxY
        zRange = z > self.minZ and z < self.maxZ    
        return xRange and yRange and zRange

    def removeLast(self):
        self.posX.pop(0)
        self.posY.pop(0)
        self.posZ.pop(0)
        self.timeList.pop(0)
        self.itemCount -= 1
    
    def appendTransform(self, t): 
        self.posX.append(t.transform.translation.x)
        self.posY.append(t.transform.translation.y)
        self.posZ.append(t.transform.translation.z)
        self.timeList.append(time.time() - self.startTime)
        self.itemCount += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = TrajectoryCalculator()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()