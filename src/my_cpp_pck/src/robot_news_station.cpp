#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsStationNode : public rclcpp::Node
{
public:
  RobotNewsStationNode() : Node("robot_news_station"), _robot_name("R2D2")
  {
    _publisher = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
    _timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&RobotNewsStationNode::publishNews, this));
  RCLCPP_INFO(this->get_logger(),"Robot News Station has been started");
  }

private:
  void publishNews()
  {
    auto msg = example_interfaces::msg::String();
    msg.data = std::string("Hi, this is ") + _robot_name + std::string(" from the Robot News Stations");
    _publisher->publish(msg);
  }
  std::string _robot_name;
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char **argv)
{
  // Initialisation of ROS 2 Communication
  rclcpp::init(argc, argv);
  // Create the node
  auto node = std::make_shared<RobotNewsStationNode>();
  /**
   * Pause the program and will keep the node alive till
   * Ctrl+C
   */
  rclcpp::spin(node);
  // ShutDown ROS 2 Communication
  rclcpp::shutdown();
  return 0;
}