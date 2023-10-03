#include <chrono>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <memory>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() : Node("image_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(16), std::bind(&ImagePublisher::captureAndPublish, this));
        init_v4l2();
    }

private:
    void init_v4l2()
    {
        // Initialization code for v4l2
        // Omitted for brevity, use ioctl calls to set up your camera
    }

    void captureAndPublish()
    {
        auto image_msg = sensor_msgs::msg::Image();
        image_msg.header.stamp = this->now();
        image_msg.height = 1080;
        image_msg.width = 1920;
        image_msg.encoding = "bgr8"; // or "rgb8" or whatever your camera produces
        image_msg.step = 1920 * 3;   // step = width * byte_per_pixel for color images

        // Capture code here to fill image_msg.data
        // Omitted for brevity

        publisher_->publish(image_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
