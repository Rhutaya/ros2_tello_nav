# ROS2 Tello Nav

## About The Project
ROS2 script to controller tello drone via remote.

## Getting Started
The driver has been tested on Ubuntu 20.04 with ROS2 Foxy.

### Prerequisites
* ROS2 Foxy
* R0S2_tello_driver ([link to repo](https://github.com/kousheekc/ros2_tello_driver))
* h264decoder ([link to repo](https://github.com/DaWelter/h264decoder))

## How to use

* clone every rep upside
* install driver for your remote controller (PS4 driver for my case)
* build your workspace 
'''
cd tello_ws
colcon build --symlink-install
'''
* And enjoy your flight :
'''
ros2 launch tello_nav full_launch.py
'''

## Contact
Arthur BOUTIGNON - arthurboutignon@gmail.com

Project Link: [https://github.com/Rhutaya/ros2_tello_nav](https://github.com/Rhutaya/ros2_tello_nav)


