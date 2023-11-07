#include "Brain.hpp"

Brain::Brain() : Node("Brain") {
    // Clients
    start_catching_client_ = this->create_client<std_srvs::srv::Trigger>("/start_catching");
    throw_client_ = this->create_client<std_srvs::srv::Trigger>("/throw_ball");

    // Tf stuff
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timers
    timer_ = this->create_wall_timer( std::chrono::milliseconds(10), std::bind(&Brain::tfCallback, this));

    wait_for_services();

    std::this_thread::sleep_for(2000ms);
    request_throw();
}

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
    //         request_throw();
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

void Brain::request_throw() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = throw_client_->async_send_request(request);
    robotState = RobotState::THROWING;

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Throw Service Call Succeeded");
        request_start_catching();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Throw Service Call Failed");
        rclcpp::shutdown();
    }
}

void Brain::wait_for_services() {
    while (!start_catching_client_->wait_for_service(std::chrono::seconds(1))) {
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