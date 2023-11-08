#include "RobotControl.hpp"

RobotControl::RobotControl() : Node("RobotControl")
{
    // Robot comm setup
    client_servo_start_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    client_servo_stop_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
    client_switch_controller_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    joint_cmd_pub_ = this->create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);
    twist_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
    launch_pub = this->create_publisher<std_msgs::msg::Bool>("/ee_launch", 10);

    // Moveit objects
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
    move_group_interface->setPlanningTime(1);
    move_group_interface->setEndEffectorLink("tool0");
    move_group_interface->startStateMonitor();
    move_group_interface->setMaxVelocityScalingFactor(1);
    move_group_interface->setMaxAccelerationScalingFactor(1);
    planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    planning_frame_id = move_group_interface->getPlanningFrame();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Sub to catching servo control
    update_ball_pred_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/ball_prediction", 10, std::bind(&RobotControl::set_catch_target, this, _1));

    // Mode 
    robot_mode = RobotControlMode::THROW;
    start_catching_service_ = this->create_service<std_srvs::srv::Trigger>("/start_catching", std::bind(&RobotControl::start_catching, this, _1, _2));
    throwing_service_ = this->create_service<std_srvs::srv::Trigger>("/throw_ball", std::bind(&RobotControl::throw_ball_request, this, _1, _2));

    // Setup
    wait_for_services();
    setup_collisions();
}

void RobotControl::setup_collisions() {
    auto col_object_table = generateCollisionObject( 2.4, 1.2, 0.04, 0.85, 0.25, -0.03, planning_frame_id, "table");
    auto col_object_backWall = generateCollisionObject( 4, 0.04, 2.0, 0.85, -0.31, 0.0, planning_frame_id, "backWall");
    auto col_object_sideWall = generateCollisionObject( 0.04, 1.2, 1.0, -0.31, 0.25, -0.5, planning_frame_id, "sideWall");

    planning_scene_interface->applyCollisionObject(col_object_table);
    planning_scene_interface->applyCollisionObject(col_object_backWall);
    planning_scene_interface->applyCollisionObject(col_object_sideWall);
}



void RobotControl::set_catch_target(geometry_msgs::msg::PoseStamped pose) {
    if (robot_mode == RobotControlMode::CATCH) {
        // pose.pose.orientation.w = -0.494082;
        // pose.pose.orientation.x = -0.492382;
        // pose.pose.orientation.y = 0.515817;
        // pose.pose.orientation.z = 0.49737;

        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform( "base_link", "ball_prediction", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "YAAAAAAAAAAAAAAAAAAAAAAAAA %s", ex.what());
            return;
        }

        auto x = t.transform.translation.x;
        auto y = t.transform.translation.y;
        auto z = t.transform.translation.z;
        if (!validTarget(x,y,z)) {
            return;
        }

        geometry_msgs::msg::Pose targetPose;
        targetPose.position.x = x;
        targetPose.position.y = y;
        targetPose.position.z = z;
        targetPose.orientation.w = -0.494082;
        targetPose.orientation.x = -0.492382;
        targetPose.orientation.y = 0.515817;
        targetPose.orientation.z = 0.49737;

        std::vector<geometry_msgs::msg::Pose> path {targetPose}; //move_group_interface->getCurrentPose("tool0").pose
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        auto frac = move_group_interface->computeCartesianPath(path, eef_step, jump_threshold, trajectory);

        bool executeSuccess = static_cast<bool>(move_group_interface->execute(trajectory));
    }
}

bool RobotControl::validTarget(double x, double y, double z) {
    return x < -0.45 && y > 0.2;
}

void RobotControl::tryMoveToTargetPose(const geometry_msgs::msg::Pose &msg) {
    RCLCPP_INFO(this->get_logger(), "Setting target pose");
    move_group_interface->setPoseTarget(msg);
    tryExecutePlan();
}

void RobotControl::tryMoveToTargetQ(const std::vector<double> &q) {
    // Convert to radians
    std::vector<double> jointTarget = qToRadians(q);

    RCLCPP_INFO(this->get_logger(), "Setting target joint position");
    move_group_interface->setJointValueTarget(jointTarget);
    tryExecutePlan();
}

std::vector<double> RobotControl::qToRadians(const std::vector<double> &q) {
    std::vector<double> radJoint;
    for (auto joint : q) {
        radJoint.push_back(joint * 3.1415 / 180.0);
    }
    return radJoint;
}

void RobotControl::tryExecutePlan() {
    moveit::planning_interface::MoveGroupInterface::Plan planMessage;

    bool planSuccess = static_cast<bool>(move_group_interface->plan(planMessage));

    // Execute movement
    if (planSuccess) {
        RCLCPP_INFO(this->get_logger(), "Move to Target Planning Succeeded! Executing...");
        bool executeSuccess = static_cast<bool>(move_group_interface->execute(planMessage));
        
        if (executeSuccess) {
            RCLCPP_INFO(this->get_logger(), "Move to Target Execution Success");
        } else {
            shutdownControl("Move to Target Execution Failure");
        }
    } else {
        shutdownControl("Move to Target Planning failed!");
    }
}

void RobotControl::shutdownControl(const std::string &errorMsg) {
    RCLCPP_ERROR(this->get_logger(), errorMsg.c_str());
    RCLCPP_ERROR(this->get_logger(), "Shutting down");
    rclcpp::shutdown();
}

void RobotControl::start_catching(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
) {
    response->success = false;
    RCLCPP_INFO(this->get_logger(), "Starting Catching");

    tryMoveToTargetQ(catching_start_joint);
    robot_mode = RobotControlMode::CATCH;

    response->success = true;
}

void RobotControl::throw_ball_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
) {
    response->success = false;
    // Go to throw pos and start servoing
    robot_mode = RobotControlMode::THROW;
    RCLCPP_INFO(this->get_logger(), "Starting throwing");
    tryMoveToTargetQ(throwing_start_joint);

    request_switch_controllers(RobotControlMode::THROW);
    request_start_servo();

    // Adjust until close to target
    auto aim_angle = M_PI;
    auto launch_angle = M_PI;
    std::string fromFrameRel = "target_tf"; 
    std::string toFrameRel = "tool0";
    geometry_msgs::msg::TransformStamped t;
    while (abs(aim_angle) > 0.5*M_PI/180.0 || abs(launch_angle) > 0.5*M_PI/180.0) {
        std::this_thread::sleep_for(1ms);

        try {
            t = tf_buffer_->lookupTransform( toFrameRel, fromFrameRel, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "YAAAAAAAAAAAAAAAAAAAAAAAAA %s", ex.what());
            break;
        }

        auto x = t.transform.translation.x;
        auto y = t.transform.translation.y;
        auto z = t.transform.translation.z; // artifically increase if needed
        aim_angle = std::atan2(x,z);
        launch_angle = std::atan2(y,z);
        // RCLCPP_INFO(this->get_logger(), "aim %f launch %f x %f y %f z %f", aim_angle, launch_angle, x,y,z);


        if (abs(aim_angle) > M_PI/3 || abs(launch_angle) > M_PI/4) {
            // RCLCPP_INFO(this->get_logger(), "angle bad");
            continue;
        }
        auto rx = std::clamp(launch_angle*10, -M_PI, M_PI);
        auto ry = std::clamp(aim_angle*10, -M_PI, M_PI);

        geometry_msgs::msg::TwistStamped twist;
        twist.header.frame_id = "tool0";
        twist.header.stamp = this->now();
        twist.twist.angular.x = -rx;
        twist.twist.angular.y = ry;
        twist_cmd_pub_->publish(twist);
    }

    // Stop Servoing
    request_stop_servo();
    request_switch_controllers(RobotControlMode::CATCH);

    // DO LAUNCH
    std_msgs::msg::Bool launchMsg;
    launchMsg.data = true;
    launch_pub->publish(launchMsg);

    robot_mode = RobotControlMode::CATCH;
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Throwing finished");

}

RobotControl::~RobotControl() {
    request_stop_servo();
    request_switch_controllers(RobotControlMode::CATCH);
}

void RobotControl::request_start_servo() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_servo_start_->async_send_request(request, std::bind(&RobotControl::client_start_servo_response_callback, this, std::placeholders::_1));
    
}

void RobotControl::request_stop_servo() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_servo_stop_->async_send_request(request, std::bind(&RobotControl::client_stop_servo_response_callback, this, std::placeholders::_1));
}

void RobotControl::request_switch_controllers(RobotControlMode mode) {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

    // Activate/Deactivate controllers based on mode
    if (mode == RobotControlMode::THROW) {
        request->activate_controllers.push_back("forward_position_controller");
        request->deactivate_controllers.push_back("joint_trajectory_controller");
    } else {
        request->activate_controllers.push_back("joint_trajectory_controller");
        request->deactivate_controllers.push_back("forward_position_controller");
    }

    // Send request
    request->strictness = 2;
    request->activate_asap = true;
    request->timeout.sec = 2.0;
    auto result = client_switch_controller_->async_send_request(request, std::bind(&RobotControl::client_switch_controller_response_callback, this, std::placeholders::_1));
}

void RobotControl::client_stop_servo_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
    if (future.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Servo Mode Off");
    } else {
        shutdownControl("Stop Servo Mode Failed");
    }
}

void RobotControl::client_start_servo_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
    if (future.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Servo Mode On");
    } else {
        shutdownControl("Start Servo Mode Failed");
    }
}

void RobotControl::client_switch_controller_response_callback(rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) 
{
    if (future.get()->ok) {
        RCLCPP_INFO(this->get_logger(), "Switching controller succeeded");
    } else {
        shutdownControl("Switching controller failed");
    }
}


void RobotControl::wait_for_services(){
    while (!client_servo_start_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for services");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    while (!client_servo_stop_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for services");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    while (!client_switch_controller_->wait_for_service(std::chrono::seconds(1))) {
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
    auto node = std::make_shared<RobotControl>();
    auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
    rclcpp::shutdown();
    return 0;
}