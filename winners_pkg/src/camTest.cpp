#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("image_publisher");
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

    cv::VideoCapture cap(0); // Open the default camera, use 0 for default, or provide the camera ID if you have multiple cameras

    if (!cap.isOpened())
    {
        RCLCPP_ERROR(node->get_logger(), "Could not open camera");
        return -1;
    }

    cv::Mat frame;
    sensor_msgs::msg::Image msg;

    msg.encoding = "bgr8";
    msg.height = 1080;
    msg.width = 1920;
    msg.step = 1920 * 3; // width * 3 bytes per pixel for BGR format

    while (rclcpp::ok())
    {
        cap >> frame;

        if (frame.empty())
        {
            RCLCPP_ERROR(node->get_logger(), "Captured frame is empty");
            break;
        }

        msg.header.stamp = node->now();
        msg.data = std::vector<uint8_t>(frame.data, frame.data + frame.total() * frame.elemSize());

        publisher->publish(msg);

        rclcpp::spin_some(node);
    }

    return 0;
}
