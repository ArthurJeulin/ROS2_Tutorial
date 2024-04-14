#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisherNode: public rclcpp::Node
{
  public:
    NumberPublisherNode(): Node("number_publisher"),_number(2)
  {
    // topic name to publish to /number
    _publisher = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    _timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&NumberPublisherNode::publishNews, this));
    RCLCPP_INFO(this->get_logger(),"Number Publisher has been started");
  }

// Publish the constant number 2
private:
  void publishNews()
  {
    auto msg = example_interfaces::msg::Int64();
    msg.data = _number ;
    RCLCPP_INFO(this->get_logger(),"%ld",msg.data);
    _publisher->publish(msg);
  }
  int _number;
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char **argv)
{
  // Initialisation of ROS 2 Communication
  rclcpp::init(argc,argv);
  // Create the node
  auto node = std::make_shared<NumberPublisherNode>();
  /**
   * Pause the program and will keep the node alive till
   * Ctrl+C 
  */
  rclcpp::spin(node);
  // ShutDown ROS 2 Communication
  rclcpp::shutdown();
  return 0;
}