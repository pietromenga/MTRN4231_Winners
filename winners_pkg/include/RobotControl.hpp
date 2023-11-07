// #pragma once

// Cpp includes
#include <memory>
#include <thread>
#include <chrono>
#include <functional>
#include <string>

// Ros2 inludes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/msg/bool.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <math.h>
#include "moveit_msgs/msg/robot_trajectory.hpp"

#include "Helpers.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

enum RobotControlMode {CATCH, THROW};

#define MAX_STEP 0.75

class RobotControl : public rclcpp::Node
{
public:
    RobotControl();
    ~RobotControl();
private:
    // Robot positions
    std::vector<double> catching_start_joint = std::vector<double>{136.8, -64.91, 117.28, -51.08, 48.33, 0.27};
    std::vector<double> throwing_start_joint = std::vector<double>{128.57, -70.04, 100.70, -29.27, 39.9, 0.03};

    // Catching target position
    geometry_msgs::msg::PoseStamped catch_target;
    rclcpp::TimerBase::SharedPtr move_catch_timer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_servo_start_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_servo_stop_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_switch_controller_;
    // Client callbacks
    void client_start_servo_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
    void client_stop_servo_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
    void client_switch_controller_response_callback(rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future);

    // Robot Control Publishers and moveit
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr launch_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pose_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
    std::string planning_frame_id;

    // Subscriptions to catch and throw topics
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr update_ball_pred_sub_;

    // Services to change robot control mode
    RobotControlMode robot_mode;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_catching_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_catching_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr throwing_service_;

    // Sets robot to servo mode and allows robot control to use catch calculation and move robot.
    void start_catching(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Sets a target for the catch function to reach
    void set_catch_target(geometry_msgs::msg::PoseStamped pose);

    // Waits for active services
    void wait_for_services();

    // Requests robot to start servo control
    void request_start_servo();

    // Requests robot to stop servo control
    void request_stop_servo();

    // Requests robot to switch between forward controller and joint controller
    void request_switch_controllers(RobotControlMode mode);
    
    // Sets up collisions for moveit to avoid
    void setup_collisions();

    // Sets a target pose and attempts movement
    void tryMoveToTargetPose(const geometry_msgs::msg::Pose &msg);
    
    // Sets a target joint pose and attempts movement
    void tryMoveToTargetQ(const std::vector<double> &q);
    
    // Tries to plan and execute target goal
    void tryExecutePlan();

    // Shutsdown the node and prints an error msg
    void shutdownControl(const std::string &errorMsg);

    // converts q pose to radians
    std::vector<double> qToRadians(const std::vector<double> &q);

    //
    void test_move();

    //
    void throw_ball_request(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};