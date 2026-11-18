cmake_minimum_required(VERSION 3.8)
project(realsense_ros_gazebo)
find_package(ament_cmake REQUIRED)
# Reuse geometry with standard ROS 2 gazebo_plugins. Historical ROS 1 src/ is not built.
install(DIRECTORY meshes xacro DESTINATION share/${PROJECT_NAME})
ament_package()
