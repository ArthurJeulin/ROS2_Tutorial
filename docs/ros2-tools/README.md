## Introduction to ROS2 Tools

### Debug and Monitor your Nodes with
```bash
ros2 run <package_name> <node_name>
# Open an terminal 1
ros2 run demo_nodes_cpp talker
# Open an terminal 2
ros2 run demo_nodes_cpp listener
```
find help with run
```bash
# get to help with run
ros2 run -h
```

```bash
# in terminal 1
source /isaac-ros-dev/install/setup.bash
ros2 run my_cpp_pkg oop_node
#in terminal 2
# list node that are running
ros2 node list
# get /cpp_test
# get info a specific node
ros2 node info <node_name>
ros2 node info /cpp_test
# get help for node
ros2 node -h
```
Sum up to command
```bash
# create a new package
ros2 pkg create
# run a node
ros2 run <package_name> <node_name>
# list all the node active
ros2 node list
# get info on one node
ros2 node info <node_name>
```
### Rename a Node at Runtime
Example:
- there is a node for a temperature sensor.
- If we need multiple temperature sensor it's need to run this node several time with a proper name for each node.
We cant launch the same node twice at the same time.  
Remap the name of the node when you are starting it:
```bash
# Open terminal 1
ros2 run my_cpp_pkg oop_node --ros-args --remap __node:=abc
# Open terminal 2
ros2 run my_cpp_pkg oop_node --ros-args -r __node:=def
# Open terminal 3
ros2 node list
```

### Colcon recap
```bash
# to build your ros2_workspace you can use colcon build
cd ros2_workspace
colcon build
# colcon build will build all the package that are into the source folder of your project
# to build only one packge
colcon build --packages-select my_cpp_pkg


# you need to make your python node code executable
chmod +x src/my_py_pkg/src/my_first_node.py
# Build your python file with --symlink-install
colcon build --packages-select my_py_pkg --symlink-install

```