# Real-Time Mapping with Boston Dynamics Spot and Microsoft HoloLens 2
## Description
Visualize the Boston Dynamics Robot Dog environment as 3D map in Augmented Reality in real-time.
## Prerequisites
- ROS Noetic
- 2 workstations
  - Ubuntu 20.04
  - Windows 10/11
## Setup
![Setup](docs/images/setup.pdf)
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
