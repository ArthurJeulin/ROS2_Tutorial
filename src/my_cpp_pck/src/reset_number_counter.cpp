#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class ResetNumberCounterNode: public rclcpp::Node
{
  public:
    ResetNumberCounterNode(): Node("reset_number_counter")
    {
      _threads.push_back(std::thread(std::bind(&ResetNumberCounterNode::callbackResetCounter,this)));

        RCLCPP_INFO(this->get_logger(),"Service server has been started.");   
    }
  private :

    void callbackResetCounter()
    {
      auto client = this->create_client<example_interfaces::srv::SetBool>("reset_counter");

      while(!client->wait_for_service(std::chrono::seconds(1)))
      {
        RCLCPP_WARN(this->get_logger(),"Waiting for the server to be up...");
      }

      auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
      
      request->data = true;

      auto future = client->async_send_request(request);
      //future.get wil block the thread until we get the response from the server.
      //can throw an error
      try
      {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(),"Result %i",(bool)response->success);
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(),"Server call failed");
      }

    }

    std::vector<std::thread> _threads;

    
};

int main(int argc, char **argv)
{
  // Initialisation of ROS 2 Communication
  rclcpp::init(argc,argv);
  // Create the node
  auto node = std::make_shared<ResetNumberCounterNode>();
  /**
   * Pause the program and will keep the node alive till
   * Ctrl+C 
  */
  rclcpp::spin(node);
  // ShutDown ROS 2 Communication
  rclcpp::shutdown();
  return 0;
}