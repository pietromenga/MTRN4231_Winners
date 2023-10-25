#include "TrajectoryCalculator.hpp"

TrajectoryCalculator::TrajectoryCalculator() : Node("TrajectoryCalculator")
{
    // Publishers
    pred_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ball_prediction", 10); 

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer( std::chrono::milliseconds(1000/*20*/), std::bind(&TrajectoryCalculator::tfCallback, this));
}

void TrajectoryCalculator::tfCallback()
{
    // Find transformation of ball relative to robot base
    std::string fromFrameRel = "base_link";
    std::string toFrameRel = "ball_tf";

    geometry_msgs::msg::TransformStamped t;

    calcTrajectory(); 

    try {
        t = tf_buffer_->lookupTransform( toFrameRel, fromFrameRel, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        // RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
    }

    // calcTrajectory();
}

void TrajectoryCalculator::calcTrajectory() {
    // TODO: DO TRAJECTORY CALC HERE BASED ON BALL_TF
    // Solve equation for first point within catching box 

    // Save and publish predicted ball end position in base frame
    geometry_msgs::msg::PoseStamped predictionPose;
    predictionPose.header.stamp = this->now();
    predictionPose.header.frame_id = "base_link";
    predictionPose.pose.position.x = -0.65;
    predictionPose.pose.position.y = 0.25;
    predictionPose.pose.position.z = 0.25;
    pred_publisher_->publish(predictionPose);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryCalculator>());
    rclcpp::shutdown();
    return 0;
}