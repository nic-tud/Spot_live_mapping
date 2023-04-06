# Real-Time Mapping with Boston Dynamics Spot and Microsoft HoloLens 2

## Description

Visualize the Boston Dynamics Robot Dog environment as 3D map in Augmented Reality in real-time.
![Pointclouds](docs/images/pointclouds.png)
![Project intention](docs/images/overview_1.png)

## Prerequisites

* ROS Noetic
* 2 workstations
  * Ubuntu 20.04 workstation with 2 network interface controllers (NIC)
  * Windows 10/11
  * optional: Microsoft HoloLens --> use emulator instead
  * optional: Boston Dynamics Spot robot --> use provided datasets instead

## Setup
### Technical setup overview
<img src="docs/images/setup.png"  width="500" height="389">

### ROS Noetic and Spot Driver from Clearpath

* run [setup.sh](src/setup.sh)

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

\--> add following line to .bashrc-file:

```
source ~/catkin_ws/devel/setup.bash
```

* edit driver.launch (change dummypassword, dummyusername, hostname) TODO: edit default IP of Spot: 192.168.80.3
  * https://support.bostondynamics.com/s/article/Spot-network-setup

```
cd ~/catkin_ws/src/spot_ros/spot_driver/launch
nano driver.launch
```

* replace [spot-driver](src/python/spot_driver) python files

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
* emulate Spot robot (use provided datasets) [Link to datasets (Google Drive)](https://drive.google.com/file/d/1VDxHfdgRFPuf-Q8Aa2ax8iOUDV3dr07L/view?usp=share_link)
  * run in third terminal:

  ```bash
  rosbag play --clock name_of_file.bag
  ```
  (--clock: causes rosbag play to publish simulated time synchronized to the messages in the bag file to the /clock topic)
  [ROS Doku Rosbag](https://wiki.ros.org/rosbag/Commandline)
### RTABMAP_ROS

* installation

```
sudo apt install ros-noetic-rtabmap-ros
```

* TODO: replace rviz file
* running launch-file 
  * using only one camera ([Link to launch-script](src/launch/rtabmap_spot_singlecam.launch)) (TODO: comment what to change for specific cam)
  
    ```
    roslaunch rtabmap_ros rtabmap_spot_singlecam.launch
    ```
  * using multiple cameras ([Link to launch-script](src/launch/rtabmap_spot_multicam.launch)) (TODO: comment what to change for specific number of cams)
  
    ```
    roslaunch rtabmap_ros rtabmap_spot_multicam.launch
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
