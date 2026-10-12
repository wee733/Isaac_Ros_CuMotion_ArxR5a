cmake_minimum_required(VERSION 3.8)
project(isaac_ros_manipulation_arx_r5a_ros2_control)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(arx5_arm_msg REQUIRED)

add_library(arx_r5a_system SHARED src/arx_r5a_system.cpp)
target_compile_features(arx_r5a_system PUBLIC cxx_std_17)
target_include_directories(arx_r5a_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(arx_r5a_system
  arx5_arm_msg
  hardware_interface
  pluginlib
  rclcpp)

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(TARGETS arx_r5a_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(arx_r5a_system)
ament_export_dependencies(
  arx5_arm_msg
  hardware_interface
  pluginlib
  rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
