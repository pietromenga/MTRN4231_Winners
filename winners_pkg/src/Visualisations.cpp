#include "Visualisations.hpp"

Visualisations::Visualisations() : Node("Visualisations") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pubmarker = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
	  timer = this->create_wall_timer(
      5000ms, std::bind(&Visualisations::setupMarkers, this));
    markerCount = 0;
    ghostTimer = this->create_wall_timer(
      100ms, std::bind(&Visualisations::ghostBall, this));
    setupMarkers();
    RCLCPP_INFO(this->get_logger(), "Adding Markers to Scene");
  
}

void Visualisations::ghostBall() {
    geometry_msgs::msg::TransformStamped t;
    try {
        t = tf_buffer_->lookupTransform( "base_link", "ball_tf", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        // RCLCPP_INFO(this->get_logger(), "YAAAAAAAAAAAAAAAAAAAAAAAAA %s", ex.what());
        return;
    }

    visualization_msgs::msg::Marker ghostBallMarker;
    
    // ghost ball
    ghostBallMarker.header.frame_id = "base_link";
    ghostBallMarker.header.stamp = this->now();
    ghostBallMarker.id = markerCount++;
    ghostBallMarker.lifetime.sec = 1.5;
    ghostBallMarker.action = visualization_msgs::msg::Marker::ADD;
    ghostBallMarker.type = visualization_msgs::msg::Marker::SPHERE;
    ghostBallMarker.frame_locked = true;
    ghostBallMarker.scale.x = 0.05;
    ghostBallMarker.scale.y = 0.05;
    ghostBallMarker.scale.z = 0.05;
    ghostBallMarker.color.r = 1.0;
    ghostBallMarker.color.g = 1.0;
    ghostBallMarker.color.b = 1.0;
    ghostBallMarker.color.a = 1.0;
    ghostBallMarker.pose.position.x = t.transform.translation.x;
    ghostBallMarker.pose.position.y = t.transform.translation.y;
    ghostBallMarker.pose.position.z = t.transform.translation.z;

    pubmarker->publish(ghostBallMarker);
}

void Visualisations::setupMarkers() {
    visualization_msgs::msg::Marker camera1, camera2, ball, catchBox, ballPred, camera_origin, target;
    
    // Camera 1 marker

    // Camera Origin marker
    camera_origin.header.frame_id = "camera_origin";
    camera_origin.header.stamp = this->now();
    camera_origin.id = markerCount++;
    camera_origin.lifetime.sec = MARKER_LIFETIME;
    camera_origin.action = visualization_msgs::msg::Marker::ADD;
    camera_origin.type = visualization_msgs::msg::Marker::CUBE;
    camera_origin.scale.x = 0.15;
    camera_origin.scale.y = 0.05;
    camera_origin.scale.z = 0.05;
    camera_origin.color.r = 0.3;
    camera_origin.color.g = 0.3;
    camera_origin.color.b = 0.3;
    camera_origin.color.a = 1.0;

    // Ball marker
    ball.header.frame_id = "ball_tf";
    ball.header.stamp = this->now();
    ball.id = markerCount++;
    ball.lifetime.sec = MARKER_LIFETIME;
    ball.action = visualization_msgs::msg::Marker::ADD;
    ball.type = visualization_msgs::msg::Marker::SPHERE;
    ball.frame_locked = true;
    ball.scale.x = 0.05;
    ball.scale.y = 0.05;
    ball.scale.z = 0.05;
    ball.color.r = 0.0;
    ball.color.g = 1.0;
    ball.color.b = 0.0;
    ball.color.a = 1.0;

    target.header.frame_id = "target_tf";
    target.header.stamp = this->now();
    target.id = markerCount++;
    target.lifetime.sec = MARKER_LIFETIME;
    target.action = visualization_msgs::msg::Marker::ADD;
    target.type = visualization_msgs::msg::Marker::SPHERE;
    target.frame_locked = true;
    target.scale.x = 0.15;
    target.scale.y = 0.01;
    target.scale.z = 0.15;
    target.color.r = 0.75;
    target.color.g = 1.0;
    target.color.b = 0.0;
    target.color.a = 1.0;

    // servo box marker
    catchBox.header.frame_id = "catch_box";
    catchBox.header.stamp = this->now();
    catchBox.id = markerCount++;
    catchBox.lifetime.sec = MARKER_LIFETIME;
    catchBox.action = visualization_msgs::msg::Marker::ADD;
    catchBox.type = visualization_msgs::msg::Marker::CUBE;
    catchBox.scale.x = 1.0;
    catchBox.scale.y = 1.0;
    catchBox.scale.z = 0.005;
    catchBox.color.r = 0.0;
    catchBox.color.g = 0.0;
    catchBox.color.b = 1.0;
    catchBox.color.a = 0.25;
    catchBox.pose.position.x = -0.25;

    // ball predict arrow
    ballPred.header.frame_id = "ball_prediction_tf";
    ballPred.header.stamp = this->now();
    ballPred.id = markerCount++;
    ballPred.lifetime.sec = MARKER_LIFETIME;
    ballPred.action = visualization_msgs::msg::Marker::ADD;
    ballPred.type = visualization_msgs::msg::Marker::SPHERE;
    ballPred.frame_locked = true;
    ballPred.scale.x = 0.05;
    ballPred.scale.y = 0.05;
    ballPred.scale.z = 0.05;
    ballPred.color.r = 1.0;
    ballPred.color.g = 0.0;
    ballPred.color.b = 0.0;
    ballPred.color.a = 0.6;

    // Publish all marker
    // markers.markers.push_back(camera1);
    // markers.markers.push_back(camera2);

    // publisher_->publish(markers);
    pubmarker->publish(camera1);
    pubmarker->publish(camera2);
    pubmarker->publish(catchBox);
    pubmarker->publish(ballPred);
    pubmarker->publish(ball);
    pubmarker->publish(target);
    pubmarker->publish(camera_origin);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Visualisations>();
//   node->setupMarkers();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}