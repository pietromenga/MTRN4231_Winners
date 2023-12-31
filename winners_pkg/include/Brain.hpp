// #pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <thread>
#include <chrono>
#include <string>

#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/bool.hpp"

#include "Helpers.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

enum RobotState {LAUNCHING, CATCHING};

class Brain : public rclcpp::Node
{
  public:
    Brain();
private:
    #define CATCH_THRESHOLD 0.25

    // Clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr launch_client;

    RobotState robotState = RobotState::CATCHING;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_throwing;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_state_sub;


    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;

    int noBallCount = 0;

    // Waits for all services to become available
    void wait_for_services();

    //
    void tfCallback();

    //
    void change_mode();

    void changeCatchState(std_msgs::msg::Bool state);
};