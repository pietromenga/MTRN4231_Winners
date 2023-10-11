#include "RobotControl.hpp"

RobotControl::RobotControl() : Node("RobotControl")
{
    // Robot comm setup
    client_servo_start_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    client_servo_stop_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
    client_switch_controller_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    joint_cmd_pub_ = this->create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);
    twist_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);

    // Moveit objects
    move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
    move_group_interface->setPlanningTime(10.0);
    planning_scene_interface = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    planning_frame_id = move_group_interface->getPlanningFrame();
    // joint_pose_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

    // Sub to catching servo control
    catch_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/catch_delta", 10, std::bind(&RobotControl::move_to_catch, this, _1));
    //throw_traj_sub_ ...

    // Mode 
    robot_mode = RobotControlMode::JOINT;
    start_catching_service_ = this->create_service<std_srvs::srv::Trigger>("/start_catching", std::bind(&RobotControl::start_catching, this, _1, _2));
    stop_catching_service_ = this->create_service<std_srvs::srv::Trigger>("/stop_catching", std::bind(&RobotControl::stop_catching, this, _1, _2));

    // Setup
    wait_for_services();
    setup_collisions();
}

void RobotControl::setup_collisions() {
    auto col_object_table = generateCollisionObject( 2.4, 1.2, 0.04, 0.85, 0.25, -0.03, planning_frame_id, "table");
    auto col_object_backWall = generateCollisionObject( 2.4, 0.04, 1.0, 0.85, -0.45, -0.7, planning_frame_id, "backWall");
    auto col_object_sideWall = generateCollisionObject( 0.04, 1.2, 1.0, -0.45, 0.25, -0.7, planning_frame_id, "sideWall");

    planning_scene_interface->applyCollisionObject(col_object_table);
    planning_scene_interface->applyCollisionObject(col_object_backWall);
    planning_scene_interface->applyCollisionObject(col_object_sideWall);
}

void RobotControl::move_to_catch(const geometry_msgs::msg::TwistStamped &twist) {
    if (robot_mode == RobotControlMode::SERVO) {
        auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>(twist);
        msg->header.stamp = this->now();
        twist_cmd_pub_->publish(std::move(msg));
    }
}

void RobotControl::tryMoveToTargetPose(const geometry_msgs::msg::Pose &msg) {
    RCLCPP_INFO(this->get_logger(), "Setting target pose");
    move_group_interface->setPoseTarget(msg);
    tryExecutePlan();
}

void RobotControl::tryMoveToTargetQ(const std::vector<double> &q) {
    // Convert to radians
    std::vector<double> jointTarget = qToRadians(q);

    RCLCPP_INFO(this->get_logger(), "Setting target joint position");
    move_group_interface->setJointValueTarget(jointTarget);
    tryExecutePlan();
}

std::vector<double> RobotControl::qToRadians(const std::vector<double> &q) {
    std::vector<double> radJoint;
    for (auto joint : q) {
        radJoint.push_back(joint * 3.1415 / 180.0);
    }
    return radJoint;
}

void RobotControl::tryExecutePlan() {
    moveit::planning_interface::MoveGroupInterface::Plan planMessage;

    bool planSuccess = static_cast<bool>(move_group_interface->plan(planMessage));

    // Execute movement
    if (planSuccess) {
        RCLCPP_INFO(this->get_logger(), "Move to Target Planning Succeeded! Executing...");
        bool executeSuccess = static_cast<bool>(move_group_interface->execute(planMessage));
        
        if (executeSuccess) {
            RCLCPP_INFO(this->get_logger(), "Move to Target Execution Success");
        } else {
            shutdownControl("Move to Target Execution Failure");
        }
    } else {
        shutdownControl("Move to Target Planning failed!");
    }
}

void RobotControl::shutdownControl(const std::string &errorMsg) {
    RCLCPP_ERROR(this->get_logger(), errorMsg.c_str());
    RCLCPP_ERROR(this->get_logger(), "Shutting down");
    rclcpp::shutdown();
}

void RobotControl::start_catching(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
) {
    response->success = false;

    tryMoveToTargetQ(catching_start_joint);

    if (robot_mode != RobotControlMode::SERVO) {
        RCLCPP_INFO(this->get_logger(), "Starting Catching");

        robot_mode = RobotControlMode::SERVO;
        request_switch_controllers(robot_mode);
        request_start_servo();

        response->success = true;
    } else {
        RCLCPP_INFO(this->get_logger(), "Already in Catching Mode");
    }
}

void RobotControl::stop_catching(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
) {
    response->success = false;

    if (robot_mode != RobotControlMode::JOINT) {
        RCLCPP_INFO(this->get_logger(), "Stopping Catching");

        robot_mode = RobotControlMode::JOINT;
        request_stop_servo();
        request_switch_controllers(robot_mode);
        response->success = true;
    } else {
        RCLCPP_INFO(this->get_logger(), "Already in Catching Mode");
    }
}

void RobotControl::request_start_servo() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_servo_start_->async_send_request(request, std::bind(&RobotControl::client_servo_response_callback, this, std::placeholders::_1));
}

void RobotControl::request_stop_servo() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_servo_stop_->async_send_request(request, std::bind(&RobotControl::client_servo_response_callback, this, std::placeholders::_1));
}

void RobotControl::request_switch_controllers(RobotControlMode mode) {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

    // Activate/Deactivate controllers based on mode
    if (mode == RobotControlMode::SERVO) {
        request->activate_controllers.push_back("forward_position_controller");
        request->deactivate_controllers.push_back("joint_trajectory_controller");
    } else {
        request->activate_controllers.push_back("joint_trajectory_controller");
        request->deactivate_controllers.push_back("forward_position_controller");
    }

    // Send request
    request->strictness = 2;
    request->activate_asap = true;
    request->timeout.sec = 2.0;
    auto result = client_switch_controller_->async_send_request(request, std::bind(&RobotControl::client_switch_controller_response_callback, this, std::placeholders::_1));
}

void RobotControl::client_servo_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
    if (future.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Servo Mode Changed");
    } else {
        shutdownControl("Servo Mode Change Failed");
    }
}

void RobotControl::client_switch_controller_response_callback(rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) 
{
    if (future.get()->ok) {
        RCLCPP_INFO(this->get_logger(), "Switching controller succeeded");
    } else {
        shutdownControl("Switching controller failed");
    }
}


void RobotControl::wait_for_services(){
    while (!client_servo_start_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for services");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    while (!client_servo_stop_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for services");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    while (!client_switch_controller_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for services");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotControl>());
  rclcpp::shutdown();
  return 0;
}