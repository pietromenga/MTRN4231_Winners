#include <chrono>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

class TrajectoryCalculator : public rclcpp::Node
{
public:
    TrajectoryCalculator();
    void tfCallback();
private:
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr catch_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pred_publisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
};