#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  // Initialize ROS2 communication
  rclcpp::init(argc,argv);
  /**
   * Create a std::shared_ptr
   * It is a RAII class so no need to use new or delete
   * To node that the node is created INSIDE the executable
   * (The executable is not the node)
   */
  auto node = std::make_shared<rclcpp::Node>("cpp_test");
  // Print a Message
  RCLCPP_INFO(node->get_logger(),"Hello Cpp Node");
  // rclcpp::spin expect a std::shared_ptr
  // Spin pause the exectuable until ctrl+c or it's requested to stop.
  rclcpp::spin(node);
  // Shutdonw the ROS2 communication
  rclcpp::shutdown();
  return 0;
}