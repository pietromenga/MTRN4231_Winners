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

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("MinimalPublisher"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("topic", 10);
        timer_ = this->create_wall_timer(
        2ms, std::bind(&MinimalPublisher::captureImg, this));

        int deviceID;
        std::cout << "Choose a /dev/video device num" << std::endl;
        std::cin >> deviceID; // 0 = open default camera. Use v4l2-ctl --list-devices to find out which device id
        int apiID = cv::CAP_ANY; 
        cap.open(deviceID, apiID);

        // check if we succeeded
        if (!cap.isOpened()) {
            std::cerr << "#### ERROR! Unable to open camera\n";
        } else {
            // Set frame rate and resolution here.
            cap.set(cv::CAP_PROP_FPS, 60);              // Set frame rate to 60 Hz
            cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);    // Set frame width
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);   // Set frame height

            // Optionally, check if the settings were applied.
            double fps = cap.get(cv::CAP_PROP_FPS);
            double width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
            double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
            std::cout << "FPS: " << fps << ", Width: " << width << ", Height: " << height << std::endl;
        }

        prevTime = std::chrono::system_clock::now();
    }

private:
    void captureImg();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    size_t count_;
    cv::VideoCapture cap;
    std::chrono::time_point<std::chrono::system_clock> prevTime;
};

void MinimalPublisher::captureImg() {

    cv::Mat frame;
    cap.read(frame);

    if (!frame.empty()) {
        cv::imshow("me", frame);
        cv::waitKey(5);
    }

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = now - prevTime;
    std::cout << "Elapsed time, " << elapsed_seconds.count() <<  " " << frame.size().height << " " << frame.size().width << std::endl;

    prevTime = now;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}