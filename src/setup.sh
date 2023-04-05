echo "###################################"
echo "### Install ROS package sources ###"
echo "###################################"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "###########################################"
echo "### Install Clearpath's package sources ###"
echo "###########################################"
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
echo "#################################"
echo "### Install ROS NOETIC (Base) ###"
echo "#################################"
sudo apt install ros-noetic-ros-base python3-rosdep -y
sudo rosdep init
sudo wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list -O /etc/ros/rosdep/sources.list.d/50-clearpath.list
rosdep update
echo "###################################"
echo "### Install Python-dependencies ###"
echo "###################################"
sudo apt update
sudo apt install -y python3-pip bridge-utils git
pip3 install cython
pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
pip3 install empy
sudo apt install qttools5-dev -y
echo "#######################################"
echo "### Building the driver from source ###"
echo "#######################################"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
source /opt/ros/noetic/setup.bash
catkin_init_workspace
git clone https://github.com/clearpathrobotics/spot_ros.git
git clone https://github.com/ros/geometry2
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src -y
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so
. ~/catkin_ws/devel/setup.bash
