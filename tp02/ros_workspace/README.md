# Installing ROS

https://wiki.ros.org/noetic/Installation/Ubuntu

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

# Setting up the environment

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

# Building

```sh
catkin_make
```

# Running

Copy `2017-10-31-22-06-52.bag` to this directory.

## Setup

```sh
roscore &
rqt &
rviz &
```

## Actual run

```sh
rosrun strdemo strdemo_node
```

Enable `/output_results` in rqt.

In rviz, 'Add' &rarr; 'By topic' &rarr; '/output_results' &rarr; 'PointCloud'.

## Play and watch `rviz`

```sh
rosbag play 2017-10-31-22-06-52.bag
```