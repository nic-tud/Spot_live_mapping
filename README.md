# Real-Time Mapping with Boston Dynamics Spot and Microsoft HoloLens 2

## Description

Visualize the Boston Dynamics Robot Dog environment as 3D map in Augmented Reality in real-time.

## Prerequisites

* ROS Noetic
* 2 workstations
  * Ubuntu 20.04 workstation with 2 network interface controllers (NIC)
  * Windows 10/11
  * optional: Microsoft HoloLens --> use emulator instead
  * optional: Boston Dynamics Spot robot --> use provided datasets instead

## Setup

### ROS Noetic and Spot Driver from Clearpath

* run [setup.sh](https://github.com/nic-tud/Spot_live_mapping/blob/main/src/setup.sh)

```
chmod +x setup.sh
./setup.sh
```

* verify successful installation with (rviz should open with a model of the Spot robot)

```
roslaunch spot_viz view_model.launch
```

* source shell script automatically in terminal (optionally, otherwise source every terminal manually)

```
nano ~/.bashrc
```
--> add following line to .bashrc-file:
```
source ~/catkin_ws/devel/setup.bash
```

* edit driver.launch (change dummypassword, dummyusername, hostname) TODO: edit default IP of Spot: 192.168.80.3
  * https://support.bostondynamics.com/s/article/Spot-network-setup

```
cd ~/catkin_ws/src/spot_ros/spot_driver/launch
nano driver.launch
```

* replace [spot-driver](https://github.com/nic-tud/Spot_live_mapping/tree/main/src/python/spot_driver) python files

```
rm -r ~/catkin_ws/src/spot_ros/spot_driver/src/spot_driver/
cp REPO_PYTHON_SRC_DIR ~/catkin_ws/src/spot_ros/spot_driver/src/
```

* replace rviz file (in original depth view, view want color view)

```
rm ~/catkin_ws/src/spot_ros/spot_viz/rviz/robot.rviz
cp REPO_RVIZ_SRC_FILE ~/catkin_ws/src/spot_ros/spot_viz/rviz/
```

* ready to visualize the robots environment (if you have an spot robot, otherwise skip to rosbag(verlinken))
  * run in one terminal:

```
roslaunch spot_driver driver.launch
```

* run in a second terminal:

```
roslaunch spot_viz view_robot.launch
```

* TODO: comment about topics (monitoring, control), comment about messages (type, hz, bw), images (rqt_graph, rviz)
* emulate Spot robot (use provided datasets) (verlinken)
  * run in third terminal:

```
rosbag play --clock name_of_file.bag
```

### RTABMAP_ROS

* installation

```
sudo apt install ros-noetic-rtabmap-ros
```

* TODO: replace rviz file
* running launch-file (verlinken)

```
roslaunch rtabmap_ros name_of_file.launch
```

[ROS Doku RTABMAP](http://wiki.ros.org/rtabmap_ros)

* comment: optional parameter (rviz:..., rtabmapviz:..., use_sim_time:..., etc.)

### rosbridge

* installation

```
sudo apt-get install ros-noetic-rosbridge-suite
```

* running rosbridge

```
roslaunch rosbridge_server rosbridge_websocket.launch
```

[ROS Doku Rosbridge](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge)

### rosbag

* record all currently running ROS topics

```
rosbag record -a
```

* play bag-file (provided dataset)
  * \--clock: causes rosbag play to publish simulated time synchronized to the messages in the bag file to the /clock topic

```
rosbag play --clock demo_mapping.bag
```

[ROS Doku Rosbag](https://wiki.ros.org/rosbag/Commandline)

![Setup](docs/images/setup.png)

* Linux workstation
  * install: ROS Noetic, RTABMAP, ROSBridge, Clearpath
  * clearpath setup --> provide shell script (important: source on startup?)
    * bosdyn sdk
    * config launch script (IP, username, password)
    * change python files of ros driver (color images, depthInVisual) --> provide file
    * change rviz --> provide file
    * comment: easy to add robot control
  * Datasets (3 rosbag files + comments on how to start + description what to see and so on)
  * RTABMAP
    * install --> provide shell script
    * change launch script (easy to define multiple cameras for user) --> provide
      * important: configs like use sim-time
      * descript launch script
    * provide rviz file
    * provide rtabmapviz file
  * ROSBridge
    * install --> include in shell script?
* Windows workstation
  * install Unity 2019.4.18f --> see YT
  * install ROS# --> see YT
  * install Pointcloud-Streaming --> see YT
  * setup scene with GameObjects, Renderer, Subscriber, Assets, IP of Linux etc.
  * TODO: MRTK - Hololens setup
