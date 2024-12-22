#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode();

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr _timer;
  int _counter;
};

MyNode::MyNode() : Node("cpp_test"), _counter(0)
{
  RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");
  // create a wall timer call the timer_callback every seconds
  _timer = this->create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&MyNode::timer_callback, this));
}
void MyNode::timer_callback()
{
  _counter++;
  RCLCPP_INFO(this->get_logger(), "Hello %d",_counter);
}

int main(int argc, char **argv)
{
  // Initialize ROS2 communication
  rclcpp::init(argc, argv);
  /**
   * Create a std::shared_ptr
   * It is a RAII class so no need to use new or delete
   * To node that the node is created INSIDE the executable
   * (The executable is not the node)
   */
  auto node = std::make_shared<MyNode>();
  // rclcpp::spin expect a std::shared_ptr
  // Spin pause the exectuable until ctrl+c or it's requested to stop.
  rclcpp::spin(node);
  // Shutdonw the ROS2 communication
  rclcpp::shutdown();
  return 0;
}