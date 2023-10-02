#include "Brain.hpp"

Brain::Brain() : Node("Brain") {
    start_catching_client_ = this->create_client<std_srvs::srv::Trigger>("/start_catching");
    stop_catching_client_ = this->create_client<std_srvs::srv::Trigger>("/stop_catching");

    keyboard_sub_ = this->create_subscription<std_msgs::msg::String>("/keyboard_input", 10, std::bind(&Brain::on_key_press, this, _1));
    test_catch_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/catch_delta", 10);

    wait_for_services();

    request_start_catching();
}

void Brain::request_start_catching() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = start_catching_client_->async_send_request(request);
}

void Brain::request_stop_catching() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = stop_catching_client_->async_send_request(request);
}

void Brain::on_key_press(const std_msgs::msg::String &keyString) {
    auto twist = geometry_msgs::msg::TwistStamped();
    twist.header.frame_id = "tool0";
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
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    while (!stop_catching_client_->wait_for_service(std::chrono::seconds(1))) {
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
    rclcpp::spin(std::make_shared<Brain>());
    rclcpp::shutdown();
    return 0;
}