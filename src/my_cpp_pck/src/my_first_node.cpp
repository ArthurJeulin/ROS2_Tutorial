#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv)
{
  // Initialisation of ROS 2 Communication
  rclcpp::init(argc,argv);
  // Create the node
  auto node = std::make_shared<rclcpp::Node>("cpp_test");
  // Print
  RCLCPP_INFO(node->get_logger(),"Hello Cpp Node");
  /**
   * Pause the program and will keep the node alive till
   * Ctrl+C 
  */
  rclcpp::spin(node);
  // ShutDown ROS 2 Communication
  rclcpp::shutdown();
  return 0;
}