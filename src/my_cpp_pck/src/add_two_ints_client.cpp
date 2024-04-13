#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode: public rclcpp::Node
{
  public:
    AddTwoIntsClientNode(): Node("add_two_ints_client")
    {
      
      _threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoInts,this,1,2)));
      _threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoInts,this,3,22)));
    }
    
    void callAddTwoInts(const int a, const int b)
    {
      auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
      while(!client->wait_for_service(std::chrono::seconds(1)))
      {
        RCLCPP_WARN(this->get_logger(),"Waiting for the server to be up...");
      }

      auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
      request->a = a;
      request->b = b;

      auto future = client->async_send_request(request);
      //future.get wil block the thread until we get the response from the server.
      //can throw an error
      try
      {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(),"%d + %d = %d",a,b, (int)response->sum);
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(),"Server call failed");
      }
    }
  private :
    std::vector<std::thread> _threads;
};

int main(int argc, char **argv)
{
  // Initialisation of ROS 2 Communication
  rclcpp::init(argc,argv);
  // Create the node
  auto node = std::make_shared<AddTwoIntsClientNode>();
  /**
   * Pause the program and will keep the node alive till
   * Ctrl+C 
  */
  rclcpp::spin(node);
  // ShutDown ROS 2 Communication
  rclcpp::shutdown();
  return 0;
}