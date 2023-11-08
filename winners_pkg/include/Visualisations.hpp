#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define MARKER_LIFETIME 5

class Visualisations : public rclcpp::Node
{
public:
    Visualisations();
    void setupMarkers();
    void ghostBall();

private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr ghostTimer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubmarker;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    int markerCount;
};