#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <functional>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TFBroadcaster : public rclcpp::Node
{
public:
    TFBroadcaster();

private:
    void tf_ball(const geometry_msgs::msg::PoseStamped & msg);
    void tf_prediction(const geometry_msgs::msg::PoseStamped & msg);
    void setupStaticTransforms();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ballpose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr velcropose_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};