#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class SmartphoneNode : public rclcpp::Node
{
public:
  SmartphoneNode() : Node("smartphone")
  {

    // Argument = topic name, Queue size, callback
    _subscriber = this->create_subscription<example_interfaces::msg::String>(
      "robot_news", 10,
      std::bind(&SmartphoneNode::callbackRobotNews, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(),"Smartphone has been started.");
  }

private:
  void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
  }
  rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr _subscriber;
};

int main(int argc, char **argv)
{
  // Initialisation of ROS 2 Communication
  rclcpp::init(argc, argv);
  // Create the node
  auto node = std::make_shared<SmartphoneNode>();
  /**
   * Pause the program and will keep the node alive till
   * Ctrl+C
   */
  rclcpp::spin(node);
  // ShutDown ROS 2 Communication
  rclcpp::shutdown();
  return 0;
}