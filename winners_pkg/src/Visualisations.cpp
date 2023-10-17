#include "Visualisations.hpp"

Visualisations::Visualisations() : Node("Visualisations") {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10); 
    markerCount = 0;
    setupMarkers();
    RCLCPP_INFO(this->get_logger(), "Adding Markers to Scene");

}

void Visualisations::setupMarkers() {
    visualization_msgs::msg::Marker camera1, camera2, ball;
    visualization_msgs::msg::MarkerArray markers;
    
    // Camera 1 marker
    camera1.header.frame_id = "camera1";
    camera1.header.stamp = this->now();
    camera1.id = markerCount++;
    camera1.ns = "object_vis";
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
    camera2.ns = "object_vis";
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
    ball.ns = "object_vis";
    ball.action = visualization_msgs::msg::Marker::ADD;
    ball.type = visualization_msgs::msg::Marker::CUBE;
    ball.scale.x = 1.0;
    ball.scale.y = 1.0;
    ball.scale.z = 1.0;
    ball.color.r = 1.0;
    ball.color.g = 0.0;
    ball.color.b = 0.0;
    ball.color.a = 1.0;

    // Publish all marker
    
    markers.markers.push_back(camera1);
    markers.markers.push_back(camera2);

    publisher_->publish(markers);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualisations>());
  rclcpp::shutdown();
  return 0;
}