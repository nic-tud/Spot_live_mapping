# Real-Time Mapping with Boston Dynamics Spot and Microsoft HoloLens 2
## Description
Visualize the Boston Dynamics Robot Dog environment as 3D map in Augmented Reality in real-time.
## Prerequisites
- ROS Noetic
- 2 workstations
  - Ubuntu 20.04 workstation with 2 network interface controllers (NIC)
  - Windows 10/11
  - optional: Microsoft HoloLens --> use emulator instead
  - optional: Boston Dynamics Spot robot --> use provided datasets instead
## Setup
### RTABMAP_ROS
- installation
```bash
sudo apt install ros-noetic-rtabmap-ros
```
- running launch-file
```bash
roslaunch rtabmap_ros xxx.launch
```
[ROS Doku RTABMAP](http://wiki.ros.org/rtabmap_ros)
- comment: optional parameter (rviz:..., rtabmapviz:..., use_sim_time:..., etc.)


### rosbridge
- installation
```bash
sudo apt-get install ros-noetic-rosbridge-suite
```
- running rosbridge
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
[ROS Doku Rosbridge](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge)


### rosbag
- record all currently running ROS topics
```bash
rosbag record -a
```
- play bag-file (provided dataset)
  - --clock: causes rosbag play to publish simulated time synchronized to the messages in the bag file to the /clock topic
```bash
rosbag play --clock demo_mapping.bag
```
[ROS Doku Rosbag](https://wiki.ros.org/rosbag/Commandline)

![Setup](docs/images/setup.png)
- Linux workstation
  - install: ROS Noetic, RTABMAP, ROSBridge, Clearpath 
  - clearpath setup --> provide shell script (important: source on startup?)
    - bosdyn sdk
    - config launch script (IP, username, password)
    - change python files of ros driver (color images, depthInVisual) --> provide file
    - change rviz --> provide file
    - comment: easy to add robot control 
  - Datasets (3 rosbag files + comments on how to start + description what to see and so on)
  - RTABMAP
    - install --> provide shell script
    - change launch script (easy to define multiple cameras for user) --> provide
      - important: configs like use sim-time
      - descript launch script
    - provide rviz file
    - provide rtabmapviz file
  - ROSBridge
    - install --> include in shell script?
- Windows workstation
  - install Unity 2019.4.18f --> see YT
  - install ROS# --> see YT
  - install Pointcloud-Streaming --> see YT
  - setup scene with GameObjects, Renderer, Subscriber, Assets, IP of Linux etc.
  - TODO: MRTK - Hololens setup
