#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode(int camera_id) : Node("camera_node_" + std::to_string(camera_id)), camera_id_(camera_id)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw_" + std::to_string(camera_id), 10);
        cap_.open(camera_id);

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera %d", camera_id);
        }
    }

    void publish_frame()
    {
        cv::Mat frame;
        sensor_msgs::msg::Image msg;

        msg.encoding = "bgr8";
        msg.height = 1080;
        msg.width = 1920;
        msg.step = 1920 * 3; // width * 3 bytes per pixel for BGR format

        cap_ >> frame;

        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Captured frame is empty");
            return;
        }

        msg.header.stamp = this->now();
        msg.data = std::vector<uint8_t>(frame.data, frame.data + frame.total() * frame.elemSize());

        publisher_->publish(msg);
    }

private:
    int camera_id_;
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{

    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    rclcpp::init(argc, argv);

    auto camera_node_0 = std::make_shared<CameraNode>(0);  // For /dev/video0
    auto camera_node_2 = std::make_shared<CameraNode>(2);  // For /dev/video2

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(camera_node_0);
    executor.add_node(camera_node_2);

    while (rclcpp::ok())
    {
        camera_node_0->publish_frame();
        camera_node_2->publish_frame();
        executor.spin_some();
    }

    rclcpp::shutdown();
    return 0;
}
