cmake_minimum_required(VERSION 3.8)
project(arx5_ros2_control)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(arx5_arm_msg REQUIRED)

include_directories(include)

add_library(arx5_system SHARED src/arx5_system.cpp)
ament_target_dependencies(arx5_system
  rclcpp pluginlib hardware_interface controller_interface arx5_arm_msg)

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(TARGETS arx5_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
