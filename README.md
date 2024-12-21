# ROS2 Humble Tutorial
### About
The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it's all open source.

## Run into a docker
### Docker on Jetson
```bash
sudo docker run --runtime nvidia -it --rm --network host --shm-size=8g \
--volume /tmp/argus_socket:/tmp/argus_socket \
--volume /etc/enctune.conf:/etc/enctune.conf \
--volume /etc/nv_tegra_release:/etc/nv_tegra_release \
--volume /tmp/nv_jetson_model:/tmp/nv_jetson_model \
--volume /var/run/dbus:/var/run/dbus \ 
--volume /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
--volume /var/run/docker.sock:/var/run/docker.sock \
--volume /ssd/jetson-containers/data:/data \
-v /etc/localtime:/etc/localtime:ro \
-v /etc/timezone:/etc/timezone:ro \
--device /dev/snd \
-e PULSE_SERVER=unix:/run/user/1000/pulse/native \
-v /run/user/1000/pulse:/run/user/1000/pulse \
--device /dev/bus/usb -e DISPLAY=:1 \
-v /tmp/.X11-unix/:/tmp/.X11-unix \
-v /tmp/.docker.xauth:/tmp/.docker.xauth \
-e XAUTHORITY=/tmp/.docker.xauth \
--device /dev/video0 \
--device /dev/video1 \
--device /dev/i2c-0 \
--device /dev/i2c-1 \
--device /dev/i2c-2 \
--device /dev/i2c-3 \
--device /dev/i2c-4 \
--device /dev/i2c-5 \
--device /dev/i2c-6 \
--device /dev/i2c-7 \
--device /dev/i2c-8 \
--device /dev/i2c-9 \
-v /run/jtop.sock:/run/jtop.sock \
--name jetson_container_20241221_145539 \
dustynv/ros:humble-desktop-l4t-r35.4.1
```
### Docker with GUI working on Jetson (easy)
```bash
cd jetson-containers
jetson-containers run $(autotag ros)
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