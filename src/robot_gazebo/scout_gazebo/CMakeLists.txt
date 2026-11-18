cmake_minimum_required(VERSION 3.8)
project(scout_gazebo)
find_package(ament_cmake REQUIRED)
ament_environment_hooks(env/cartographer_runtime.dsv)
# Also supply core Lua includes in our config directory: the local binary's
# compiled-in /opt path need not exist on machines without sudo installation.
find_path(CARTOGRAPHER_LUA_DIR map_builder.lua PATHS
  "${CMAKE_INSTALL_PREFIX}/../cartographer_runtime/opt/ros/humble/share/cartographer/configuration_files"
  "/opt/ros/$ENV{ROS_DISTRO}/share/cartographer/configuration_files")
if(CARTOGRAPHER_LUA_DIR)
  install(DIRECTORY "${CARTOGRAPHER_LUA_DIR}/" DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.lua")
endif()
install(DIRECTORY config meshes models urdf worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py")
install(PROGRAMS scripts/pointcloud_to_scan.py scripts/check_simulation.py
  scripts/apriltag_landmarks.py scripts/save_map.py scripts/initial_pose.py
  scripts/check_cartographer.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
