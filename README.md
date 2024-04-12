# franka-gh-ros1-backend

This is a ROS1 Neotic node which link the follwing Robot plugin in Grasshopper with Franka Panda Arm: 

https://github.com/BmadeRobots/Robots

## Prerequisite
- Install Ubuntu 20.04 with RT Kernel. We tested the kernel version 5.15.148-rt74, you can build your own kernel by following this [link](https://www.acontis.com/en/building-a-real-time-linux-kernel-in-ubuntu-preemptrt.html) 
- Install ROS Neotic [link](http://wiki.ros.org/noetic/Installation/Ubuntu)
- Install Frankx, we are using a version from [Toni-SM](https://github.com/Toni-SM) because it allow us to listen to the robot state while the robot is moving. See this (PR)[https://github.com/pantor/frankx/pull/44]
```
git clone --recurse-submodules -b robot-state https://github.com/heiyin1207/frankx.git
cd frankx
mkdir -p build
cd build
cmake -DBUILD_TYPE=Release ..
make
sudo make install
cd ..
pip install .
```
## Usage
```
source /opt/ros/noetic/setup.bash
git clone https://github.com/BmadeRobots/franka-gh-ros1-backend.git
cd franka-gh-ros1-backend
catkin build
source ./devel/setup.bash 
roslaunch rosbridge_server rosbridge_websocket.launch
```

On the second terminal:
```
cd franka-gh-ros1-backend
source ./devel/setup.bash 
rosrun franka_client main.py 
```
