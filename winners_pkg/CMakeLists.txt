cmake_minimum_required(VERSION 3.8)
project(winners_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Packages for our project
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED )
find_package(tf2)
find_package(tf2_ros)
find_package(tf2_geometry_msgs)
find_package(control_msgs)
find_package(moveit_ros_planning_interface)
# find_package(ros_ur_driver)
find_package(moveit_visual_tools)
find_package(controller_manager_msgs)
find_package(trajectory_msgs)


# Inclue dirs
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# TESTING CAMERA
add_executable(camTest src/camTest.cpp)
ament_target_dependencies(camTest 
  rclcpp std_msgs sensor_msgs geometry_msgs OpenCV
)

# BALL POSE
add_executable(ballPose src/ballPose.cpp)
ament_target_dependencies(ballPose 
  rclcpp std_msgs sensor_msgs geometry_msgs OpenCV
)

# VELCRO PAD POSE
add_executable(velcroPadPose src/velcroPadPose.cpp)
ament_target_dependencies(velcroPadPose 
  rclcpp std_msgs sensor_msgs geometry_msgs OpenCV
)

# TRANSFORMS
add_executable(transforms src/transforms.cpp)
ament_target_dependencies(transforms 
  rclcpp std_msgs geometry_msgs OpenCV tf2 tf2_ros tf2_geometry_msgs
)

# ROBOT CONTROL 
add_executable(RobotControl src/RobotControl.cpp)
ament_target_dependencies(RobotControl 
  rclcpp std_msgs geometry_msgs control_msgs moveit_ros_planning_interface moveit_visual_tools controller_manager_msgs
  trajectory_msgs
)

# BRAIN
add_executable(Brain src/Brain.cpp)
ament_target_dependencies(Brain 
  rclcpp std_msgs geometry_msgs control_msgs moveit_ros_planning_interface moveit_visual_tools controller_manager_msgs
  trajectory_msgs
)

# DUAL CAMERAS
add_executable(dual_cam_test src/dual_cam_test.cpp)
ament_target_dependencies(dual_cam_test rclcpp)


# add_executable(ballPose src/ballPose.cpp)
# ament_target_dependencies(ballPose rclcpp std_msgs geometry_msgs OpenCV)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  camTest
  ballPose
  velcroPadPose
  transforms
  RobotControl
  DESTINATION lib/${PROJECT_NAME})


ament_package()
