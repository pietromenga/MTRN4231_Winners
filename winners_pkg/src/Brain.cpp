#include "Brain.hpp"

Brain::Brain() : Node("Brain") {
    // Clients
    launch_client = this->create_client<std_srvs::srv::Trigger>("/launch_ball");

    // Tf stuff
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timers
    timer_ = this->create_wall_timer( std::chrono::milliseconds(10), std::bind(&Brain::tfCallback, this));

    robot_state_sub = create_subscription<std_msgs::msg::Bool>("/catch_state", 10, std::bind(&Brain::changeCatchState, this, _1));

    wait_for_services();
    std::this_thread::sleep_for(2000ms);
    change_mode();
}

void Brain::changeCatchState(std_msgs::msg::Bool state) {
    RCLCPP_INFO(this->get_logger(), "State Changed");

    if (state.data) {
        robotState = RobotState::CATCHING;
    } else {
        robotState = RobotState::LAUNCHING;
    }
}

void Brain::tfCallback()
{
    if (robotState != RobotState::CATCHING) {
        return;
    } 

    if (noBallCount >= 500) {
        change_mode();
    }

    // Get transformation between ball and end effector 
    std::string fromFrameRel = "ball_tf"; 
    std::string toFrameRel = "base_link";

    geometry_msgs::msg::TransformStamped t;

    try {
        t = tf_buffer_->lookupTransform( toFrameRel, fromFrameRel, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        // RCLCPP_INFO(this->get_logger(), "No frames?");
        noBallCount++;
        return;
    }

    noBallCount = 0;

    // Distance to end eff
    // auto distance = std::sqrt(std::pow(t.transform.translation.x,2) + std::pow(t.transform.translation.y,2) + std::pow(t.transform.translation.z,2));
    auto xrange = t.transform.translation.x < -0.1 && t.transform.translation.x > -0.7;
    auto yrange = t.transform.translation.y > 0.1 && t.transform.translation.y < 0.5;
    auto zrange = t.transform.translation.z > -0.5 && t.transform.translation.z < 1.0;

    // RCLCPP_INFO(this->get_logger(), "xrange: %d, yrange: %d, zrange: %d", xrange, yrange, zrange);

    // If within distance stop catching and initiate throw
    if (xrange && yrange && zrange) {
        robotState = RobotState::LAUNCHING;
        std::this_thread::sleep_for(4000ms);
        change_mode();
    }
}

void Brain::change_mode() {
    RCLCPP_INFO(this->get_logger(), "Request Changing Mode");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = launch_client->async_send_request(request);
}

void Brain::wait_for_services() {
    while (!launch_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for services");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Brain>());
  rclcpp::shutdown();
  return 0;
}