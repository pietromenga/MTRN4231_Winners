#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Pose, TwistStamped, PoseStamped, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray, Float32MultiArray, Bool
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
from spatialmath.base import q2r, r2q
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
from  enum import Enum
import time
from builtin_interfaces.msg import Duration
import time
#import moveit_com

JOINT_ORDER = [5, 0, 1, 2, 3, 4]
CATCHING_JOINTS = [136.8, -64.91, 117.28, -51.08, 48.33, 0.27]
LAUNCHING_JOINT = [128.57, -70.04, 100.70, -29.27, 39.9, 0.03]

class RobotMode(Enum):
    LAUNCH = 0
    CATCH = 1

class Control(Node):
    def __init__(self):
        super().__init__('Control')
        # set up subscribers
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', self.jointStateCallback, 10)
        self.ball_pred_sub = self.create_subscription(PoseStamped, '/ball_prediction', self.moveToPrediction, 10)
        
        # set up publishers
        self.joint_pub = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.goal_pub = self.create_publisher(Float32MultiArray, "/joint_goal", 10)
        self.state_pub = self.create_publisher(Bool, "/catch_state", 10)
        self.arduino = self.create_publisher(Bool, "/arduino", 10)
        self.twist_cmd_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

        # services
        self.srv = self.create_service(Trigger, '/launch_ball', self.launchBall)

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot kinematics
        self.ur5 = rtb.models.UR5()
        self.pose = Pose()

        # Parameters
        self.speed = 0.75 #m/s
        self.targetGoalBound = 0.5 #degrees
        self.robotMode = RobotMode.CATCH
        self.sumX, self.sumY, self.sumZ = 0.0, 0.0, 0.0
        self.validCount = 0
        self.validTimer = time.time()

        # self.scene = moveit_commander.PlanningSceneInterface()
        # self.setupCollisions()

        # Clients
        self.servo_start = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.servo_start.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.servo_stop = self.create_client(Trigger, '/servo_node/stop_servo')
        while not self.servo_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.switch_controller = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.switch_controller.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    # def setupCollisions(self):
    #     box_pose = PoseStamped()
    #     box_pose.header.frame_id = "base_link"
    #     box_pose.pose.position.x = 0.85
    #     box_pose.pose.position.y = 0.25
    #     box_pose.pose.position.z = -0.03
    #     self.scene.add_box("table", box_pose, size=(2.4, 1.2, 0.04))

    def servo_start_request(self):
        self.get_logger().info('Starting servo')
        req = Trigger.Request()
        future = self.servo_start.call_async(req)
        time.sleep(1.0)

    def servo_stop_request(self):
        self.get_logger().info('Stopping servo')
        req = Trigger.Request()
        future = self.servo_stop.call_async(req)
        time.sleep(1.0)

    def switch_controller_request(self):
        req = SwitchController.Request()
        self.get_logger().info('Switching controller')

        if self.robotMode == RobotMode.CATCH:
            req.activate_controllers.append("joint_trajectory_controller")
            req.deactivate_controllers.append("forward_position_controller")
        else:
            req.activate_controllers.append("forward_position_controller")
            req.deactivate_controllers.append("joint_trajectory_controller")

        req.strictness = 2
        req.activate_asap = True
        timeout = Duration()
        timeout.sec = 2
        req.timeout = timeout
        future = self.switch_controller.call_async(req)
        time.sleep(1.0)

    def sendState(self):
        self.get_logger().info('Sending state to brain')
        boolmsg = Bool()
        boolmsg.data = bool(self.robotMode.value)
        self.state_pub.publish(boolmsg)

    def aimAtTarget(self):
        # Aiming control loop
        aim_angle = np.pi
        launch_angle = np.pi
        while abs(aim_angle) > self.targetGoalBound*np.pi/180.0 or abs(launch_angle) > self.targetGoalBound*np.pi/180.0:
            time.sleep(0.001)
            
            try:
                t = self.tf_buffer.lookup_transform(
                    "tool0",
                    "target_tf",
                    rclpy.time.Time())
            except Exception as ex:
                return
            
            # Get angles
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z 
            aim_angle = np.arctan2(x,z)
            launch_angle = np.arctan2(y,z)

            # Target too far from curr aim
            if abs(aim_angle) > np.pi/3.0 or abs(launch_angle) > np.pi/4.0:
                continue
            
            # Pub to servo
            rx = np.clip(launch_angle*10.0, -np.pi, np.pi)
            ry = np.clip(aim_angle*10.0, -np.pi, np.pi)
            twist = TwistStamped()
            twist.header.frame_id = "tool0"
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.angular.x = -rx
            twist.twist.angular.y = ry
            self.twist_cmd_pub.publish(twist)

    def launchBall(self, request, response: Trigger.Response):
        if self.robotMode == RobotMode.CATCH:
            self.get_logger().info("Commencing Launch")
            self.robotMode = RobotMode.LAUNCH
            self.sendState()

            # Move to start and swap to servo
            self.sendQ(LAUNCHING_JOINT)
            self.switch_controller_request()
            self.servo_start_request()

            # Shoot
            self.aimAtTarget()
            boolmsg = Bool()
            boolmsg.data = True
            self.arduino.publish(boolmsg)

            # Go back to catching
            self.get_logger().info("Launching finished")
            self.robotMode = RobotMode.CATCH
            self.servo_stop_request()
            self.switch_controller_request()
            self.sendQ(CATCHING_JOINTS)
            self.sendState()

            response.success = True
            return response
        
        response.success = False
        return response

    def moveToPrediction(self, data):
        if self.robotMode != RobotMode.CATCH:
            return
        
        try:
            t1 = self.tf_buffer.lookup_transform(
                "base_link",
                "ball_prediction_tf",
                rclpy.time.Time())
            t2 = self.tf_buffer.lookup_transform(
                "base_link",
                "tool0",
                rclpy.time.Time())
        except Exception as ex:
            # self.get_logger().info(
            #     f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        x = t1.transform.translation.x - t2.transform.translation.x
        y = t1.transform.translation.y - t2.transform.translation.y
        z = t1.transform.translation.z - t2.transform.translation.z
        
        goalT = Pose()
        goalT.position = self.pose.position
        goalT.orientation = self.pose.orientation
        goalT.position.x += x
        goalT.position.y += y
        goalT.position.z += z

        if not self.checkValid(goalT.position.x,goalT.position.y,goalT.position.z):
            return
        
        # if validTimer has not been renewed in 0.5 sec reset avg
        if (time.time() - self.validTimer) > 0.5:
            self.resetAverages()
        goalT.position = self.averagePosition(goalT.position) # Valid position, store and average

        distance = np.sqrt(x**2 + y**2 + z**2)
        moveTime = distance / self.speed   
        self.sendPose(goalT, moveTime)

    def resetAverages(self):
        self.sumX, self.sumY, self.sumZ = 0,0,0
        self.validCount = 0
        self.validTimer = time.time()

    def averagePosition(self, position: Point):
        self.sumX += position.x
        self.sumY += position.y
        self.sumZ += position.z
        self.validCount += 1
        newPos = Point()
        newPos.x = self.sumX / self.validCount
        newPos.y = self.sumY / self.validCount
        newPos.z = self.sumZ / self.validCount
        self.validTimer = time.time()
        return newPos

    def checkValid(self, x,y,z):
        distance = np.sqrt(x**2 + y**2 + z**2)   
        validDistance = distance > 0.45 and distance < 0.85
        validZ = z > 0.0
        validX = x < -0.3
        validY = y > 0.0
        valid = (validDistance and validX and validY and validZ)

        if not valid:
            self.get_logger().info(f"Failed validity check {x}, {y}, {z}")

        return valid

    def inverseKin(self, pose: Pose):
        trans = [pose.position.x, pose.position.y, pose.position.z]
        rot = q2r([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
        targetPose = sm.SE3.Rt(rot, trans)
        qNear = np.deg2rad(np.array(CATCHING_JOINTS)) # 
        qTarget =  self.ur5.ikine_LM(targetPose, q0=self.ur5.q, joint_limits=True)

        return qTarget.q

    def jointStateCallback(self, data):
        if self.robotMode != RobotMode.CATCH:
            return
        
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

    def sendPose(self, pose: Pose, time):
        qTarget = self.inverseKin(pose)
        qTarget = [float(joint) for joint in qTarget]

        self.get_logger().info(f"PUBLISHING JOINT GOAL {[np.degrees(j) for j in qTarget]} in time {time}")

        jointTraj = JointTrajectory()
        jointTraj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        jointTrajPoint = JointTrajectoryPoint()
        jointTrajPoint.positions = qTarget
        jointTrajPoint.velocities = []
        jointTrajPoint.accelerations = []
        jointTrajPoint.effort = []

        if time > 1.0:
            jointTrajPoint.time_from_start.nanosec = int(float(0.95) * 1000000000)
        else:
            jointTrajPoint.time_from_start.nanosec = int(float(time) * 1000000000)


        jointTraj.points = [jointTrajPoint]
        self.joint_pub.publish(jointTraj)

        jointGoal = Float32MultiArray()
        jointGoal.data = qTarget
        self.goal_pub.publish(jointGoal)

    def sendQ(self, qTarget):
        qTarget = [float(np.radians(joint)) for joint in qTarget]

        self.get_logger().info(f"PUBLISHING JOINT GOAL {[np.degrees(j) for j in qTarget]} in time {0.9}")

        jointTraj = JointTrajectory()
        jointTraj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        jointTrajPoint = JointTrajectoryPoint()
        jointTrajPoint.positions = qTarget
        jointTrajPoint.velocities = []
        jointTrajPoint.accelerations = []
        jointTrajPoint.effort = []

        jointTrajPoint.time_from_start.nanosec = int(0.9 * 1000000000)

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
