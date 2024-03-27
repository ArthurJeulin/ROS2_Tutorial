#include "rclcpp/rclcpp.hpp"

class MyCustomNode: public rclcpp::Node
{
  public:
    MyCustomNode(): Node("node_name")
    {
      
    }
  private :
    
};

int main(int argc, char **argv)
{
  // Initialisation of ROS 2 Communication
  rclcpp::init(argc,argv);
  // Create the node
  auto node = std::make_shared<MyCustomNode>();
  /**
   * Pause the program and will keep the node alive till
   * Ctrl+C 
  */
  rclcpp::spin(node);
  // ShutDown ROS 2 Communication
  rclcpp::shutdown();
  return 0;
}