cmake_minimum_required(VERSION 3.0.2)
project(ros_oa)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf rospy std_msgs genmsg)



## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(OA src/OA.cpp)
target_link_libraries(OA ${catkin_LIBRARIES})


