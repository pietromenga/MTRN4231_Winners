#include "rclcpp/rclcpp.hpp"

class DualCamTest : public rclcpp::Node
{
public:
    DualCamTest() : Node("dual_cam_test")
    {
        RCLCPP_INFO(this->get_logger(), "DualCamTest node has started.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualCamTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
