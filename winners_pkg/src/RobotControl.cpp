#include "RobotControl.hpp"

RobotControl::RobotControl() : Node("RobotControl")
{

    // Moveit comm setup
    client_servo_start_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    client_servo_stop_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
    client_switch_controller_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    joint_cmd_pub_ = this->create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);
    twist_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
    RCLCPP_INFO(this->get_logger(), "BEFORE");


    move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
    move_group_interface->setPlanningTime(10.0);
    // joint_pose_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

    RCLCPP_INFO(this->get_logger(), "AFTER");

    wait_for_services();

    // Sub to catching servo control
    catch_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/catch_delta", 10, std::bind(&RobotControl::move_to_catch, this, _1));
    //throw_traj_sub_ ...

    // Mode 
    robot_mode = RobotControlMode::JOINT;
    start_catching_service_ = this->create_service<std_srvs::srv::Trigger>("/start_catching", std::bind(&RobotControl::start_catching, this, _1, _2));
    stop_catching_service_ = this->create_service<std_srvs::srv::Trigger>("/stop_catching", std::bind(&RobotControl::stop_catching, this, _1, _2));
}

void RobotControl::move_to_catch(const geometry_msgs::msg::TwistStamped &twist) {
    if (robot_mode == RobotControlMode::SERVO) {
        auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>(twist);
        msg->header.stamp = this->now();
        twist_cmd_pub_->publish(std::move(msg));
    }
}

void RobotControl::test_move() {
    RCLCPP_INFO(this->get_logger(), "Starting test");

    moveit::planning_interface::MoveGroupInterface::Plan planMessage;

    // moveit::core::RobotStatePtr current_state = move_group_interface->getCurrentState();
    // moveit::core::JointModelGroupPtr joint_model_group = current_state->getJointModelGroup("ur_manipulator");

    // std::vector<double> joint_group_positions = {
    //     0.09529498, -1.392773, 1.5180874, -0.13439, 1.83119945, -0.62412974
    // };
    // // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // // joint_group_positions[0] = -1.0;  // radians
    // move_group_interface->setJointValueTarget(joint_group_positions);

    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.0;
    msg.orientation.y = 1.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.200;
    msg.position.y = -0.692;
    msg.position.z = 0.034;

    move_group_interface->setPoseTarget(msg);
    bool success = static_cast<bool>(move_group_interface->plan(planMessage));

    //Execute movement to point 1
    if (success) {
      move_group_interface->execute(planMessage);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
    
}

void RobotControl::start_catching(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
) {
    (void) request;


    RCLCPP_INFO(this->get_logger(), "Starting Catching");

    robot_mode = RobotControlMode::SERVO;
    request_switch_controllers(robot_mode);
    request_start_servo();

    response->success = true;
}

void RobotControl::stop_catching(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
) {
    (void) request;
    test_move();

    RCLCPP_INFO(this->get_logger(), "Stopping Catching");

    robot_mode = RobotControlMode::JOINT;
    request_stop_servo();
    request_switch_controllers(robot_mode);

    response->success = true;
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
        RCLCPP_INFO(this->get_logger(), "Servo Service call succeeded");
        // actived_servo_flag = true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Servo Service call failed");
    }
}

void RobotControl::client_switch_controller_response_callback(rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) 
{
    if (future.get()->ok) {
        RCLCPP_INFO(this->get_logger(), "Switch controller Service call succeeded");
        // switched_controllers_flag = true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Switch controller Service call failed");
    }
}


void RobotControl::wait_for_services(){
    while (!client_servo_start_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    while (!client_servo_stop_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    while (!client_switch_controller_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
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