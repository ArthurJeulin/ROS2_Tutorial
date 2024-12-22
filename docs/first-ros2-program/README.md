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