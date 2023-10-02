// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class VelcroPadPublisher : public rclcpp::Node
{
public:
    VelcroPadPublisher()
    : Node("minimal_publisher2"), count_(0)
    {
        // Pub to ball pose
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("velcro_pad_pose", 10); 

        // Sub from camera output
        cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&VelcroPadPublisher::find_pad, this, _1));
    }

private:
    void find_pad(const sensor_msgs::msg::Image & img);
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
    size_t count_;
};

void VelcroPadPublisher::find_pad(const sensor_msgs::msg::Image & img) {
    // Read img in from sub and process ball pose

    geometry_msgs::msg::Pose padPose;
    publisher_->publish(padPose);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelcroPadPublisher>());
    rclcpp::shutdown();
    return 0;
}