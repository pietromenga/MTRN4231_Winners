// #pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <memory>
#include <thread>
#include <chrono>
#include <functional>
#include <string>

#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "Helpers.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Brain : public rclcpp::Node
{
public:
    Brain();
private:

    // Clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_catching_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_catching_client_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_sub_; // Testing for now
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr test_catch_twist_pub_;

    // Function for testing topics
    void on_key_press(const std_msgs::msg::String &keyString);

    // Waits for all services to become available
    void wait_for_services();

    // Sends a request to the robot control node to start moving the robot for catching
    void request_start_catching();

    // Sends a request to the robot control node to stop moving the robot for catching
    void request_stop_catching();

    // Handles service responses and stops program if any fail
    void service_response_handler(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
};