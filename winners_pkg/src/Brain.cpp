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
    if (state.data) {
        robotState = RobotState::CATCHING;
    } else {
        robotState = RobotState::LAUNCHING;
    }
}

void Brain::tfCallback()
{
    // Get transformation between ball and end effector 
    std::string fromFrameRel = "ball_tf"; 
    std::string toFrameRel = "tool0";

    geometry_msgs::msg::TransformStamped t;

    try {
        t = tf_buffer_->lookupTransform( toFrameRel, fromFrameRel, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        return;
    }

    // Distance to end eff
    auto distance = std::sqrt(std::pow(t.transform.translation.x,2) + std::pow(t.transform.translation.y,2) + std::pow(t.transform.translation.z,2));

    // If within distance stop catching and initiate throw
    if (distance < CATCH_THRESHOLD && robotState == RobotState::CATCHING) {
        change_mode();
    }
}

void Brain::change_mode() {
    RCLCPP_INFO(this->get_logger(), "Requesting Throw");

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