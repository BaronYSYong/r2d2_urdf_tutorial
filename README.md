# README #

## Reference
* http://wiki.ros.org/urdf_tutorial
* https://github.com/ros/urdf_tutorial.git

## Installation of ROS in Ubuntu 16.04
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
