#include "Brain.hpp"

Brain::Brain() : Node("Brain") {
    start_catching_client_ = this->create_client<std_srvs::srv::Trigger>("/start_catching");
    stop_catching_client_ = this->create_client<std_srvs::srv::Trigger>("/stop_catching");
    throw_client_ = this->create_client<std_srvs::srv::Trigger>("/throw_ball");

    keyboard_sub_ = this->create_subscription<std_msgs::msg::String>("/keyboard_input", 10, std::bind(&Brain::on_key_press, this, _1));
    test_catch_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/catch_delta", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer( std::chrono::milliseconds(10), std::bind(&Brain::tfCallback, this));

    wait_for_services();

    std::this_thread::sleep_for(2000ms);
    request_throw();
}

// void Brain::generateJointPose(std::vector<double> jointQ) {
//     joint_trajectory_msg = trajectory_msgs::msg::JointTrajectory();
//     joint_trajectory_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
//                                         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

//     joint_trajectory_point = JointTrajectoryPoint()
//     joint_trajectory_point.positions = [q1,q2,q3,q4,q5,q6]
//     joint_trajectory_point.velocities = []
//     joint_trajectory_point.accelerations = []
//     joint_trajectory_point.effort = []

//     joint_trajectory_msg.
// }

void Brain::tfCallback()
{
    // Get transformation between ball and end effector 
    std::string fromFrameRel = "ball_tf"; 
    std::string toFrameRel = "tool0";

    geometry_msgs::msg::TransformStamped t;

    try {
        t = tf_buffer_->lookupTransform( toFrameRel, fromFrameRel, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        // RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
    }

    // // Distance to end eff
    // auto distance = std::sqrt(std::pow(t.transform.translation.x,2) + std::pow(t.transform.translation.y,2) + std::pow(t.transform.translation.z,2));

    // // If within distance stop catching and initiate throw
    // if (distance < CATCH_THRESHOLD && robotState == RobotState::CATCHING) {
    //     // Transition to throw mode and attemp a throw 
    //     request_stop_catching();
    //     if (robotState == RobotState::THROWING) {
    //         request_throw();
    //     }
    // }
}

void Brain::request_start_catching() {
    RCLCPP_INFO(this->get_logger(), "Requesting to start catching...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = start_catching_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Request Start Catching Call Succeeded");
        robotState = RobotState::CATCHING;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Request Start Catching Call Failed");
        rclcpp::shutdown();
    }
}

void Brain::request_stop_catching() {
    RCLCPP_INFO(this->get_logger(), "Requesting to stop catching...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = stop_catching_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Request Stop Catching Call Succeeded");
        robotState = RobotState::THROWING;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Request Stop Catching Call Failed");
        rclcpp::shutdown();
    }
}

void Brain::request_throw() {
    RCLCPP_INFO(this->get_logger(), "Requesting a throw from robot control");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = throw_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Throw Service Call Succeeded");
        request_start_catching();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Throw Service Call Failed");
        rclcpp::shutdown();
    }
}

void Brain::on_key_press(const std_msgs::msg::String &keyString) {
    auto twist = geometry_msgs::msg::TwistStamped();
    twist.header.frame_id = "base_link";
    twist.header.stamp = this->now();
    
    if (keyString.data == "up") {
        twist.twist.linear.x = 0.1;
    } else if (keyString.data == "down") {
        twist.twist.linear.x = -0.1;
    } else if (keyString.data == "left") {
        twist.twist.linear.y = -0.1;
    } else if (keyString.data == "right") {
        twist.twist.linear.y = 0.1;
    }

    test_catch_twist_pub_->publish(twist);
}

void Brain::wait_for_services() {
    while (!start_catching_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for services");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    while (!stop_catching_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for services");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    while (!throw_client_->wait_for_service(std::chrono::seconds(1))) {
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
  rclcpp::spin(std::make_shared<Brain>());
  rclcpp::shutdown();
  return 0;
}