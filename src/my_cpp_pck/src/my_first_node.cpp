#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node
{
  public:
    MyNode(): Node("cpp_test"),_counter(0)
    {
      // Print
      RCLCPP_INFO(this->get_logger(),"Hello Cpp Node");
      _timer = this->create_wall_timer(std::chrono::seconds(1),
                          std::bind(&MyNode::timerCallback,this));
    }
  private :
    void timerCallback(){
      RCLCPP_INFO(this->get_logger(),"Hello %d",_counter);
      ++_counter;
    }
    rclcpp::TimerBase::SharedPtr _timer;
    int _counter;
};

int main(int argc, char **argv)
{
  // Initialisation of ROS 2 Communication
  rclcpp::init(argc,argv);
  // Create the node
  auto node = std::make_shared<MyNode>();
  /**
   * Pause the program and will keep the node alive till
   * Ctrl+C 
  */
  rclcpp::spin(node);
  // ShutDown ROS 2 Communication
  rclcpp::shutdown();
  return 0;
}