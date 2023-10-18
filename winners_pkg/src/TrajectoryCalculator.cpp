#include "TrajectoryCalculator.hpp"

TrajectoryCalculator::TrajectoryCalculator() : Node("TrajectoryCalculator")
{
    // Publishers
    pred_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("ball_prediction", 10); 
    catch_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("catch_delta", 10); 

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer( std::chrono::milliseconds(20), std::bind(&TrajectoryCalculator::tfCallback, this));
}

void TrajectoryCalculator::tfCallback()
{
    // Check if the transformation is between "map" and "dynamic_frame"
    std::string fromFrameRel = "base_link";
    std::string toFrameRel = "ball_pose";

    geometry_msgs::msg::TransformStamped t;

    try {
        t = tf_buffer_->lookupTransform( toFrameRel, fromFrameRel, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
    }

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryCalculator>());
    rclcpp::shutdown();
    return 0;
}