#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoInstsServerNode: public rclcpp::Node
{
  public:
    AddTwoInstsServerNode(): Node("add_two_ints_server")
    {
      _server = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
        std::bind(&AddTwoInstsServerNode::callbackAddTwoInts, this,
        std::placeholders::_1, std::placeholders::_2));
        /**
         * std::placeholders::_1 est là pour le premier argument de la callback(request)
         * std::placeholders::_2 est là pour le deuxième argument de la callback(response)
        */
       RCLCPP_INFO(this->get_logger(),"Service server has been started.");   
    }
  private :
    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
      const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
      {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(),"%ld + %ld = %ld",request->a, request->b, response->sum);
      }
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr _server;
    
};

int main(int argc, char **argv)
{
  // Initialisation of ROS 2 Communication
  rclcpp::init(argc,argv);
  // Create the node
  auto node = std::make_shared<AddTwoInstsServerNode>();
  /**
   * Pause the program and will keep the node alive till
   * Ctrl+C 
  */
  rclcpp::spin(node);
  // ShutDown ROS 2 Communication
  rclcpp::shutdown();
  return 0;
}