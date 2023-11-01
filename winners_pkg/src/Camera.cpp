#include "Camera.hpp"

Camera::Camera() : Node("Camera") {
    this->declare_parameter("camera_id", 0);
    camera_id = this->get_parameter("camera_id").as_int();
    RCLCPP_INFO(this->get_logger(), "Camera ID: %d", camera_id);

    std::string topic = "camera" + std::to_string(camera_id) + "/image_raw";

    pubImage = this->create_publisher<sensor_msgs::msg::Image>(topic, 10);
    timer = this->create_wall_timer(
      10ms, std::bind(&Camera::pubCamera, this));

    cap = cv::VideoCapture(camera_id * 2); // vid cap 0 and 2

    if (!cap.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open camera");
        rclcpp::shutdown();
    }

}

void Camera::pubCamera() {
    cap >> frame;

    if (frame.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Captured frame is empty");
        return;
    }

    cv_bridge::CvImage msg;
    msg.header.stamp = this->now(); // Same timestamp and tf frame as input image
    msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    msg.image    = frame; // Your cv::Mat
    pubImage->publish(*msg.toImageMsg());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Camera>());
  rclcpp::shutdown();
  return 0;
}