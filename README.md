# ROS2 Humble Tutorial
### About
The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it's all open source.

## Run into a docker
### Docker on Jetson
```bash
sudo docker run --runtime nvidia -it --rm --network=host docker.io/dustynv/ros:humble-desktop-l4t-r35.4.1
```
## Check ROS2 Installation
Open in terminal 1
```bash
ros2 run demo_nodes_cpp talker
```
Open in terminal 2
```bash
ros2 run demo_nodes_cpp listener
```