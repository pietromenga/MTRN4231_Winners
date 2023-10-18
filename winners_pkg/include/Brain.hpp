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

using namespace std::chrono_literals;
using std::placeholders::_1;

class Brain : public rclcpp::Node
{
  public:
    Brain();
  private:

};