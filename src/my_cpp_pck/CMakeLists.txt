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
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
add_executable(add_two_ints_client src/add_two_ints_client.cpp)
add_executable(number_publisher src/number_publisher.cpp)
add_executable(number_counter src/number_counter.cpp)
add_executable(number_counter_reset src/number_counter_reset.cpp)
add_executable(reset_number_counter src/reset_number_counter.cpp)

ament_target_dependencies(cpp_node rclcpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)
ament_target_dependencies(smartphone rclcpp example_interfaces)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)
ament_target_dependencies(number_publisher rclcpp example_interfaces)
ament_target_dependencies(number_counter rclcpp example_interfaces)
ament_target_dependencies(number_counter_reset rclcpp example_interfaces)
ament_target_dependencies(reset_number_counter rclcpp example_interfaces)

install(TARGETS
  cpp_node
  robot_news_station
  smartphone
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client
  number_publisher
  number_counter
  number_counter_reset
  reset_number_counter
  DESTINATION lib/${PROJECT_NAME})
# lib is inside the install folder in the learn_ros2 workspace
ament_package()


