cmake_minimum_required(VERSION 3.8)
project(my_cpp_pck)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

# name of the executable cpp_node
add_executable(cpp_node src/my_first_node.cpp)
add_executable(robot_news_station src/robot_news_station.cpp)
add_executable(smartphone src/smartphone.cpp)

ament_target_dependencies(cpp_node rclcpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)
ament_target_dependencies(smartphone rclcpp example_interfaces)

install(TARGETS
  cpp_node
  robot_news_station
  smartphone
  DESTINATION lib/${PROJECT_NAME})
# lib is inside the install folder in the learn_ros2 workspace
ament_package()


