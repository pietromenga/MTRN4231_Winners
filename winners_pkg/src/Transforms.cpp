#include "Transforms.hpp"

TFBroadcaster::TFBroadcaster() : Node("TFBroadcaster")
{
    ballpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "ball_pose", 10, std::bind(&TFBroadcaster::tf_ball, this, _1));
    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "shoot_target", 10, std::bind(&TFBroadcaster::tf_target, this, _1));
    pred_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "ball_prediction", 10, std::bind(&TFBroadcaster::tf_prediction, this, _1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    setupStaticTransforms();
    RCLCPP_INFO(this->get_logger(), "Adding Transforms to Scene");
}

// Dynamic transform the ball to the world space
void TFBroadcaster::tf_ball(const geometry_msgs::msg::PoseStamped & msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.pose.position.x);

    geometry_msgs::msg::TransformStamped transform_msg;

    transform_msg.header.stamp = msg.header.stamp;
    transform_msg.header.frame_id = "camera_origin";
    transform_msg.child_frame_id = "ball_tf";
    transform_msg.transform.translation.x = msg.pose.position.x;
    transform_msg.transform.translation.y = msg.pose.position.y;
    transform_msg.transform.translation.z = msg.pose.position.z;
    transform_msg.transform.rotation.x = msg.pose.orientation.x;
    transform_msg.transform.rotation.y = msg.pose.orientation.y;
    transform_msg.transform.rotation.z = msg.pose.orientation.z;
    transform_msg.transform.rotation.w = msg.pose.orientation.w;

    tf_broadcaster_->sendTransform(transform_msg);
}

void TFBroadcaster::tf_target(const geometry_msgs::msg::PoseStamped & msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.pose.position.x);

    geometry_msgs::msg::TransformStamped transform_msg;

    transform_msg.header.stamp = msg.header.stamp;
    transform_msg.header.frame_id = "camera_origin";
    transform_msg.child_frame_id = "target_tf";
    transform_msg.transform.translation.x = msg.pose.position.x;
    transform_msg.transform.translation.y = msg.pose.position.y;
    transform_msg.transform.translation.z = msg.pose.position.z;
    transform_msg.transform.rotation.x = msg.pose.orientation.x;
    transform_msg.transform.rotation.y = msg.pose.orientation.y;
    transform_msg.transform.rotation.z = msg.pose.orientation.z;
    transform_msg.transform.rotation.w = msg.pose.orientation.w;

    tf_broadcaster_->sendTransform(transform_msg);
}

// Dynamic transform the velcro to the world space
void TFBroadcaster::tf_prediction(const geometry_msgs::msg::PoseStamped & msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.pose.position.x);

    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = msg.header.stamp;
    transform_msg.header.frame_id = "camera_origin";
    transform_msg.child_frame_id = "ball_prediction_tf";
    transform_msg.transform.translation.x = msg.pose.position.x;
    transform_msg.transform.translation.y = msg.pose.position.y;
    transform_msg.transform.translation.z = msg.pose.position.z;
    transform_msg.transform.rotation.x = msg.pose.orientation.x;
    transform_msg.transform.rotation.y = msg.pose.orientation.y;
    transform_msg.transform.rotation.z = msg.pose.orientation.z;
    transform_msg.transform.rotation.w = msg.pose.orientation.w;

    tf_broadcaster_->sendTransform(transform_msg);
}

void TFBroadcaster::setupStaticTransforms() {
	geometry_msgs::msg::TransformStamped camera_origin, catch_box, bucket;

    camera_origin.header.frame_id = "base_link";
    camera_origin.child_frame_id = "camera_origin";
    camera_origin.transform.translation.x = 1.020;
    camera_origin.transform.translation.y = 0.740;
    camera_origin.transform.translation.z = 0.494;
    camera_origin.transform.rotation.w = 0.707;
    camera_origin.transform.rotation.x = 0.0;
    camera_origin.transform.rotation.y = 0.0;
    camera_origin.transform.rotation.z = 0.707;

    // Camera2 static position and orientation
    catch_box.header.frame_id = "base_link";
    catch_box.child_frame_id = "catch_box";
    catch_box.transform.translation.x = -0.5;
    catch_box.transform.translation.y = 0.25;
    catch_box.transform.translation.z = 0.12;
    catch_box.transform.rotation.w = 1.0;
    catch_box.transform.rotation.x = 0.0;
    catch_box.transform.rotation.y = 0.0;
    catch_box.transform.rotation.z = 0.0;

    bucket.header.frame_id = "tool0";
    bucket.child_frame_id = "bucket";
    bucket.transform.translation.x = 0.0;
    bucket.transform.translation.y = 0.0;
    bucket.transform.translation.z = 0.145;
    bucket.transform.rotation.w = 1.0;
    bucket.transform.rotation.x = 0.0;
    bucket.transform.rotation.y = 0.0;
    bucket.transform.rotation.z = 0.0;

    tf_static_broadcaster_->sendTransform(catch_box);
    tf_static_broadcaster_->sendTransform(camera_origin);
    tf_static_broadcaster_->sendTransform(bucket);
}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}