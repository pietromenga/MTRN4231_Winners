#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <memory>
#include <thread>
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
    TFBroadcaster()
    : Node("TFBroadcaster")
    {
      ballpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "ball_pose", 10, std::bind(&TFBroadcaster::tf_ball, this, _1));
      velcropose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "velcro_pad_pose", 10, std::bind(&TFBroadcaster::tf_velcro, this, _1));
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      setupStaticTransforms();
    }

  private:
    void tf_ball(const geometry_msgs::msg::PoseStamped & msg);
    void tf_velcro(const geometry_msgs::msg::PoseStamped & msg);
    void setupStaticTransforms();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ballpose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr velcropose_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

// Dynamic transform the ball to the world space
void TFBroadcaster::tf_ball(const geometry_msgs::msg::PoseStamped & msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.pose.position.x);

    geometry_msgs::msg::TransformStamped transform_msg;
    tf_broadcaster_->sendTransform(transform_msg);
}

// Dynamic transform the velcro to the world space
void TFBroadcaster::tf_velcro(const geometry_msgs::msg::PoseStamped & msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.pose.position.x);

    geometry_msgs::msg::TransformStamped transform_msg;
    tf_broadcaster_->sendTransform(transform_msg);
}

void TFBroadcaster::setupStaticTransforms() {
    // put all static transforms in here
}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}