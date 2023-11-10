#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray, Float32MultiArray, Bool

import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
from spatialmath.base import q2r, r2q

JOINT_ORDER = [5, 0, 1, 2, 3, 4]
CATCHING_JOINTS = [136.8, -64.91, 117.28, -51.08, 48.33, 0.27]

class Control(Node):
    def __init__(self):
        super().__init__('Control')
        #set up subscribers
        self.ee_rot_sub = self.create_subscription(Bool, '/move_test', self.doTest, 10)
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', self.jointStateCallback, 10)

        #set up publishers
        self.joint_pub = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.goal_pub = self.create_publisher(Float32MultiArray, "/joint_goal", 10)

        self.ur5 = rtb.models.UR5()
        self.pose = Pose()

    def doTest(self, data):
        goalT = Pose()
        goalT.position = self.pose.position
        goalT.orientation = self.pose.orientation
        goalT.position.z += 0.2

        self.sendPose(goalT)

    def inverseKin(self, pose: Pose):
        trans = [pose.position.x, pose.position.y, pose.position.z]
        rot = q2r([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
        targetPose = sm.SE3.Rt(rot, trans)
        qNear = np.deg2rad(np.array(CATCHING_JOINTS)) # 
        qTarget =  self.ur5.ikine_LM(targetPose, q0=qNear, joint_limits=True)

        return qTarget.q

    def jointStateCallback(self, data):
        self.ur5.q = np.array([data.position[5],data.position[0],data.position[1],data.position[2],data.position[3],data.position[4]])
        self.getPose()

    def getPose(self):
        currentPose = self.ur5.fkine(self.ur5.q)
        self.pose.position.x = currentPose.t[0]
        self.pose.position.y = currentPose.t[1]
        self.pose.position.z = currentPose.t[2]
        quaternion = r2q(currentPose.R)
        self.pose.orientation.w = quaternion[0]
        self.pose.orientation.x = quaternion[1]
        self.pose.orientation.y = quaternion[2]
        self.pose.orientation.z = quaternion[3]
        # self.get_logger().info(f"CURRENT POSE: x:{self.pose.position.x:.3f}, y:{self.pose.position.y:.3f}, z:{self.pose.position.z:.3f}, qx:{self.pose.orientation.x:.3f}, qy:{self.pose.orientation.y:.3f}, qz:{self.pose.orientation.z:.3f}, ")

    def sendPose(self, pose: Pose):
        qTarget = self.inverseKin(pose)
        qTarget = [float(joint) for joint in qTarget]

        jointTraj = JointTrajectory()
        jointTraj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        jointTrajPoint = JointTrajectoryPoint()
        jointTrajPoint.positions = qTarget
        jointTrajPoint.velocities = []
        jointTrajPoint.accelerations = []
        jointTrajPoint.effort = []

        self.get_logger().info(f"PUBLISHING JOINT GOAL {[np.degrees(j) for j in qTarget]}")

        jointTraj.points = [jointTrajPoint]
        self.joint_pub.publish(jointTraj)

        jointGoal = Float32MultiArray()
        jointGoal.data = qTarget
        self.goal_pub.publish(jointGoal)

def main(args=None):
    rclpy.init(args=args)
    node = Control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
