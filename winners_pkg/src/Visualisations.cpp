#include "Visualisations.hpp"

Visualisations::Visualisations() : Node("Visualisations") {
    pubmarker = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
	timer = this->create_wall_timer(
      1000ms, std::bind(&Visualisations::setupMarkers, this));
    markerCount = 0;
    setupMarkers();
    RCLCPP_INFO(this->get_logger(), "Adding Markers to Scene");

}

void Visualisations::setupMarkers() {
    visualization_msgs::msg::Marker camera1, camera2, ball, catchBox;
    visualization_msgs::msg::MarkerArray markers;
    
    // Camera 1 marker
    camera1.header.frame_id = "camera1";
    camera1.header.stamp = this->now();
    camera1.id = markerCount++;
    camera1.action = visualization_msgs::msg::Marker::ADD;
    camera1.type = visualization_msgs::msg::Marker::CUBE;
    camera1.scale.x = 0.1;
    camera1.scale.y = 0.1;
    camera1.scale.z = 0.1;
    camera1.color.r = 1.0;
    camera1.color.g = 1.0;
    camera1.color.b = 1.0;
    camera1.color.a = 1.0;
    camera1.pose.position.z = camera1.scale.z / 2.0;

    // Camera 2 marker
    camera2.header.frame_id = "camera2";
    camera2.header.stamp = this->now();
    camera2.id = markerCount++;
    camera2.action = visualization_msgs::msg::Marker::ADD;
    camera2.type = visualization_msgs::msg::Marker::CUBE;
    camera2.scale.x = 0.1;
    camera2.scale.y = 0.1;
    camera2.scale.z = 0.1;
    camera2.color.r = 1.0;
    camera2.color.g = 1.0;
    camera2.color.b = 1.0;
    camera2.color.a = 1.0;
    camera2.pose.position.z = camera2.scale.z / 2.0;

    // Ball marker
    ball.header.frame_id = "ball_tf";
    ball.header.stamp = this->now();
    ball.id = markerCount++;
    ball.action = visualization_msgs::msg::Marker::ADD;
    ball.type = visualization_msgs::msg::Marker::CUBE;
    ball.frame_locked = true;
    ball.scale.x = 1.0;
    ball.scale.y = 1.0;
    ball.scale.z = 1.0;
    ball.color.r = 1.0;
    ball.color.g = 0.0;
    ball.color.b = 0.0;
    ball.color.a = 1.0;

    // servo box marker
    catchBox.header.frame_id = "catch_box";
    catchBox.header.stamp = this->now();
    catchBox.id = markerCount++;
    catchBox.lifetime.sec = 1;
    catchBox.action = visualization_msgs::msg::Marker::ADD;
    catchBox.type = visualization_msgs::msg::Marker::CUBE;
    catchBox.scale.x = 1.0;
    catchBox.scale.y = 1.0;
    catchBox.scale.z = 1.0;
    catchBox.color.r = 0.0;
    catchBox.color.g = 0.0;
    catchBox.color.b = 1.0;
    catchBox.color.a = 0.25;
    catchBox.pose.position.z = catchBox.scale.z / 2.0;
    catchBox.pose.position.x = -0.25;

    // Publish all marker
    // markers.markers.push_back(camera1);
    // markers.markers.push_back(camera2);

    // publisher_->publish(markers);
    pubmarker->publish(camera1);
    pubmarker->publish(camera2);
    pubmarker->publish(catchBox);
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