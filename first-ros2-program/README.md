## Introduction

#### Install Colcon & Setup Autocompletion
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