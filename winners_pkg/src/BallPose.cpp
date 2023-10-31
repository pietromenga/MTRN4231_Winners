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
// #include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class BallPosePublisher : public rclcpp::Node
{
public:
    BallPosePublisher()
    : Node("BallPosePublisher"), count_(0)
    {
        // Pub to ball pose
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ball_pose", 10); 

        // Sub from camera output
        cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&BallPosePublisher::find_ball, this, _1));
    }

private:
    void find_ball(const sensor_msgs::msg::Image & img);
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
    size_t count_;
};

void BallPosePublisher::find_ball(const sensor_msgs::msg::Image & img) {
    // Read img in from sub and process ball pose
    
    geometry_msgs::msg::PoseStamped ballPose;
    ballPose.header.stamp = this->now();
    ballPose.pose.position.x = -0.6;
    ballPose.pose.position.y = 0.3;
    ballPose.pose.position.z = 0.2;
    publisher_->publish(ballPose);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallPosePublisher>());
    rclcpp::shutdown();
    return 0;
}