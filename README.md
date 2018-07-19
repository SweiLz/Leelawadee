# Leelawadee
ROS Robot indoor navigation 

## ROS Packages for Leelawadee
|Version|Kinetic + Ubuntu Xenial|Melodic + Ubuntu Bionic|
|:---:|:---:|:---:|
|1.0.0|-|-|


## ROS Packages dependencies

1. Clone from Git
```
cd ~/catkin_ws/src
git clone https://github.com/SweiLz/Leelawadee.git
git clone https://github.com/EAIBOT/ydlidar.git
```

1. Install all package dependencies
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```
> sudo apt install ros-kinetic-urdf
> sudo apt install ros-kinetic-tf
> sudo apt install ros-kinetic-xacro

1. Make
```
cd ~/catkin_ws
catkin_make
```