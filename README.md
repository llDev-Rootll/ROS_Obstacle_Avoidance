
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

  
#  ROS based turtlebot3 obstacle avoidance


A simple ROS based algorithm for a turtlebot3 to avoid obstacles
## System Requirements
- Ubuntu 18.04 (LTS)
- ROS Melodic
- Turtlebot3 ROS Packages
## Installing required Turtlebot3 Packages
In a terminal run :

    sudo apt-get install ros-melodic-turtlebot3 ros-melodic-turtlebot3-msgs
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc


## Steps to build the package

  Make a catkin workspace catkin_ws and run the following commands :
  

    cd <path_to_ws>/catkin_ws/src
    git clone https://github.com/llDev-Rootll/ROS_Obstacle_Avoidance.git
    cd ../
    catkin_make

## Steps to run using the launch file

In a terminal run :

    source devel/setup.bash
    roslaunch ros_oa tb_oa.launch record:=true
where record is the command line argument for enabling rosbag recording of ~ 15 seconds. 

Use record:=false to disable recording, for example:

    roslaunch ros_oa tb_oa.launch record:=false
## Inspecting the collected rosbag file

In the results directory run the following command to inspect the rosbag:

    rosbag info ros_bag_pub_sub.bag
## Seeing the rosbag replay in action
With the gazebo not running and a roscore running , run the following command to play the recorded rosbag:

     rosbag play ros_bag_pub_sub.bag

## Running cpplint & cppcheck tests
Run the following command in the root directory to generate cpplint results in **results** folder
 
    sh run_cpplint.sh
Run the following command in the root directory to generate cppcheck results in **results** folder

    sh run_cppcheck.sh

  

