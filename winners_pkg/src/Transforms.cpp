#include "Transforms.hpp"

TFBroadcaster::TFBroadcaster() : Node("TFBroadcaster")
{
    ballpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "ball_pose", 10, std::bind(&TFBroadcaster::tf_ball, this, _1));
    velcropose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "velcro_pad_pose", 10, std::bind(&TFBroadcaster::tf_velcro, this, _1));
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
    tf_broadcaster_->sendTransform(transform_msg);
}

// Dynamic transform the velcro to the world space
void TFBroadcaster::tf_velcro(const geometry_msgs::msg::PoseStamped & msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.pose.position.x);

    geometry_msgs::msg::TransformStamped transform_msg;
    tf_broadcaster_->sendTransform(transform_msg);
}

void TFBroadcaster::setupStaticTransforms() {
	geometry_msgs::msg::TransformStamped camera1_static, camera2_static, catch_box;

    // Camera1 static position and orientation
    camera1_static.header.frame_id = "base_link";
    camera1_static.child_frame_id = "camera1";
    camera1_static.transform.translation.x = 1.0;
    camera1_static.transform.translation.y = 0.0;
    camera1_static.transform.translation.z = 0.0;
    camera1_static.transform.rotation.w = 1.0;
    camera1_static.transform.rotation.x = 0.0;
    camera1_static.transform.rotation.y = 0.0;
    camera1_static.transform.rotation.z = 0.0;

    // Camera2 static position and orientation
    camera2_static.header.frame_id = "base_link";
    camera2_static.child_frame_id = "camera2";
    camera2_static.transform.translation.x = 0.0;
    camera2_static.transform.translation.y = 1.0;
    camera2_static.transform.translation.z = 0.0;
    camera2_static.transform.rotation.w = 1.0;
    camera2_static.transform.rotation.x = 0.0;
    camera2_static.transform.rotation.y = 0.0;
    camera2_static.transform.rotation.z = 0.0;

    // Camera2 static position and orientation
    catch_box.header.frame_id = "base_link";
    catch_box.child_frame_id = "catch_box";
    catch_box.transform.translation.x = -0.5;
    catch_box.transform.translation.y = 0.25;
    catch_box.transform.translation.z = 0.0;
    catch_box.transform.rotation.w = 1.0;
    catch_box.transform.rotation.x = 0.0;
    catch_box.transform.rotation.y = 0.0;
    catch_box.transform.rotation.z = 0.0;

    tf_static_broadcaster_->sendTransform(camera1_static);
    tf_static_broadcaster_->sendTransform(camera2_static);
    tf_static_broadcaster_->sendTransform(catch_box);
}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}