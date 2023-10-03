#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() : Node("imagePublisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

        // Initialize V4L2
        init_v4l2();

        timer_ = this->create_wall_timer(
            16ms, std::bind(&ImagePublisher::captureAndPublish, this));
    }

private:
    void init_v4l2()
    {
        fd = open("/dev/video0", O_RDWR);
        if (fd == -1) {
            perror("Opening video device");
            return;
        }

        // TODO: Add more settings here like frame rate, resolution, etc.

        // Initialize buffer
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        // TODO: Fill other buf fields as necessary

        // Memory mapping
        buffer = mmap(NULL, 
                      /* length of the mapped area */,
                      PROT_READ | PROT_WRITE,
                      MAP_SHARED,
                      fd, 
                      buf.m.offset);

        if (buffer == MAP_FAILED) {
            perror("Memory mapping failed");
            return;
        }

        // Queue the buffer
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Queue buffer");
            return;
        }

        // Start capturing
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
            perror("Start capture");
            return;
        }
    }

    void captureAndPublish()
    {
        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            perror("Retrieve frame");
            return;
        }

        // Convert this to ROS2 sensor_msgs::msg::Image and publish.
        sensor_msgs::msg::Image image_msg;
        image_msg.header.stamp = this->now();
        image_msg.height = /* height */;
        image_msg.width = /* width */;
        image_msg.encoding = "bgr8";  
        image_msg.step = /* step */;
        image_msg.data = std::vector<uint8_t>((uint8_t*)buffer, (uint8_t*)buffer + buf.length);

        publisher_->publish(image_msg);

        // Requeue buffer
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Queue buffer");
            return;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    int fd;
    void *buffer;
    struct v4l2_buffer buf;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
