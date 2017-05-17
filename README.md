# README #

## Environment
* Ubuntu 16.04 (Xenial)
* ROS Kinetic

## Reference
* http://wiki.ros.org/urdf_tutorial
* https://github.com/ros/urdf_tutorial.git
* URDF in Gazebo
    * http://gazebosim.org/tutorials/?tut=ros_urdf

## Installation of ROS Kinetic in Ubuntu 16.04
http://wiki.ros.org/kinetic/Installation/Ubuntu
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ sudo apt-get install python-rosinstall
```
## Build
```
$ cd /path/to/catkin_ws/src
$ git clone https://github.com/BaronYSYong/r2d2_urdf_tutorial.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

## Command list
* $ roslaunch r2d2_urdf_tutorial display.launch model:='$(find r2d2_urdf_tutorial)/urdf/01-myfirst.urdf'
* $ roslaunch r2d2_urdf_tutorial display.launch model:='$(find r2d2_urdf_tutorial)/urdf/02-multipleshapes.urdf'
* $ roslaunch r2d2_urdf_tutorial display.launch model:='$(find r2d2_urdf_tutorial)/urdf/03-origins.urdf'
* $ roslaunch r2d2_urdf_tutorial display.launch model:='$(find r2d2_urdf_tutorial)/urdf/04-materials.urdf'
* $ roslaunch r2d2_urdf_tutorial display.launch model:='$(find r2d2_urdf_tutorial)/urdf/05-visual.urdf'
* $ roslaunch r2d2_urdf_tutorial display.launch model:='$(find r2d2_urdf_tutorial)/urdf/06-flexible.urdf'
* $ roslaunch r2d2_urdf_tutorial display.launch model:='$(find r2d2_urdf_tutorial)/urdf/07-physics.urdf'
* $ roslaunch r2d2_urdf_tutorial xacrodisplay.launch model:='$(find r2d2_urdf_tutorial)/urdf/08-macroed.urdf.xacro'
* $ roslaunch r2d2_urdf_tutorial gazebo.launch model:='$(find r2d2_urdf_tutorial)/urdf/08-macroed.urdf.xacro'
* $ roslaunch r2d2_urdf_tutorial gazebo.launch model:='$(find r2d2_urdf_tutorial)/urdf/09-roscontrol.urdf.xacro'
* $ roslaunch r2d2_urdf_tutorial control.launch
