#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class NumberCounterNode: public rclcpp::Node
{
  public:
    NumberCounterNode(): Node("number_counter"),_total_number(0),_reset(false)
  {
    // publish to number_count
    _publisher = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
    // _timer = this->create_wall_timer(std::chrono::milliseconds(500),
    //                                  std::bind(&NumberCounterNode::publishNews, this));
  // Argument = topic name, Queue size, callback
    _subscriber = this->create_subscription<example_interfaces::msg::Int64>(
        "number", 10,
        std::bind(&NumberCounterNode::callbackNumberCounter, this, std::placeholders::_1));

    _server = this->create_service<example_interfaces::srv::SetBool>("reset_counter",
        std::bind(&NumberCounterNode::callbackResetCounter, this,
        std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(),"Number Counter has been started");
  }

private:
  void publishNews()
  {
    auto msg = example_interfaces::msg::Int64();
    msg.data = _total_number;
    _publisher->publish(msg);
  }

  void callbackNumberCounter(const example_interfaces::msg::Int64::SharedPtr msg)
  {

    if(_reset == true)
    {
      RCLCPP_INFO(this->get_logger(),"Requested to Reset Counter : %i",_reset);
      _total_number = 0;
      _reset = false;
    }


    _total_number+= msg->data;
    auto newdata = example_interfaces::msg::Int64();
    newdata.data = _total_number;
    _publisher->publish(newdata);
    RCLCPP_INFO(this->get_logger(),"%f",_total_number);
  }

  void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
      const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
      if(request->data == true)
      {
        _reset = true;
        request->data  = false;
        response->success = true;
        RCLCPP_INFO(this->get_logger(),"Receive from server %i, Response %i",request->data, response->success);
      }
    }
  

  double _total_number;
  bool _reset ;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr _server;
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr _publisher;
  rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr _subscriber;
  rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char **argv)
{
  // Initialisation of ROS 2 Communication
  rclcpp::init(argc,argv);
  // Create the node
  auto node = std::make_shared<NumberCounterNode>();
  /**
   * Pause the program and will keep the node alive till
   * Ctrl+C 
  */
  rclcpp::spin(node);
  // ShutDown ROS 2 Communication
  rclcpp::shutdown();
  return 0;
}