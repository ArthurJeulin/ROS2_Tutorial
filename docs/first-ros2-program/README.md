## Introduction to ROS2

#### Install Colcon & Setup Auto-Completion
If you are not using the devcontainer
```bash
sudo apt-get install -y python3-colcon-common-extensions
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
#### Create a ROS2 Workspace

```bash
mkdir -p /isaac-ros-dev/src
# all the code we will create will be in the src of this workspace
cd /isaac-ros-dev
# Check the build works
colcon build
# We can source our workspace to make available the node
source install/local_setup.bash
# if you want to source your workspace and on what the workspace depend on 
source install/setup.bash
```
#### Write a C++ Package
```bash
# go to the src directory of your workspace
cd /isaac-ros-dev/src
# create a new package
ros2 pkg create <package_name> \
# build system
--build-type ament_cmake \
# packages that your new package depend on 
--dependencies rclcpp
```
Create my_cpp_pkg
```bash
cd /isaac-ros-dev/src 
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
cd /isaac-ros-dev
colcon build
# to build only my_cpp_pck
colcon build --packages-select my_cpp_pkg
```

#### Write a Python Package (doesn't work for now)
To create a ROS2 node, we need to create a package firstly. Packages allow us to seperate our code into reusable blocks. Each package is an independent unit. For example, there will have a package that handle:
- Camera driver
- Wheels kinematics
- Motion planning  
Let's create a python package in your ROS2 workspace.
```bash
# go to the src directory of your workspace
cd /isaac-ros-dev/src
# create a new package
ros2 pkg create <package_name> \
# build system
--build-type ament_python \
# packages that your new package depend on 
--dependencies rclpy
```
Create my_py_pkg 
```bash
cd /isaac-ros-dev/src
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy --license Apache-2.0
```
To compile the new package created
```bash
cd /isaac-ros-dev
colcon build
```

#### What is a ROS2 Node ?
A node is a sub-part of our application and should have a single purpose.  
The application will contain many nodes which will be put into packages.
Node will communicate with each other.  
A package is an independent unit inside our application.
We will create node inside the package.  
Example:
- Camera pkg will handle the camera has an independent unit.  
  - Camera driver to get the frame
  - Image processing algorithm
  - Computer Vision algorithm  
All those programs are nodes. Each node can be launched separately.
- Motion planning pkg
  - Motion planning
  - Path correction
- Hardware control pkg
  - State publisher
  - Main Control loop
  - Drivers   

##### Nodes definition
- Subprograms of the application, responsible of on item.
- Conbined in to a graph
- Commnicate with each other through topics, services, and parameters  

Benefits:
- Reduce code complexity
- Fault tolerance
- Can be written in Python or C++ and both nodes can communicate together.

#### Write a C++ Node - Minimal Code
```bash
# Assume you already create a cpp package named my_cpp_pkg and colcon build work
cd /isaac-ros-dev/src/my_cpp_pkg/src
touch my_first_node.cpp
```
Now edit my_first_node.cpp
```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  // Initialize ROS2 communication
  rclcpp::init(argc,argv);
  /**
   * Create a std::shared_ptr
   * It is a RAII class so no need to use new or delete
   * To node that the node is created INSIDE the executable
   * (The executable is not the node)
   */
  auto node = std::make_shared<rclcpp::Node>("cpp_test");
  // Print a Message
  RCLCPP_INFO(node->get_logger(),"Hello Cpp Node");
  // rclcpp::spin expect a std::shared_ptr
  // Spin pause the exectuable until ctrl+c or it's requested to stop.
  rclcpp::spin(node);
  // Shutdonw the ROS2 communication
  rclcpp::shutdown();
  return 0;
}
```
Need to compile the program. Update CMakeLists.txt with:
```C
# name of the executable cpp_node
add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)
# ${PROJECT_NAME} is my_cpp_pkg
install(TARGETS 
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
  )
```
build with colcon
```bash
cd /isaac-ros-dev
colcon build --packages-select my_cpp_pkg
# Path to the executable that will just build
ls install/my_cpp_pkg/lib/my_cpp_pkg/cpp_node 
# Run the node
./install/my_cpp_pkg/lib/my_cpp_pkg/cpp_node
```
Open a new terminal
```bash
source /isaac-ros-dev/install/setup.bash 
ros2 run my_cpp_pkg cpp_node
```
#### ROS2 - Language libraries
- rcl stands for ros2 client library.
- rcl is a pure library with all the core functionalities.
  - Under RCL there is the ros2 middleware with DDS means data distribution service. This is handling all the communication in the application.
  - rcl is the lowest ros2 client library that can be use and it is the bridge with ros2 middlware.
- rclcpp is a cpp client library build on top of rcl which provides a binding with rcl functionalities.
- rclpy is a python client library build on top of rcl that can be use into python application.