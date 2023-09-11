cmake_minimum_required(VERSION 3.0.2)
project(franka_move)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    move_dynamic_planning_interface
    rviz_visual_tools
    moveit_visual_tools
    move_dynamic_msgs
    actionlib
    control_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
    LIBRARIES
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning_interface
        move_dynamic_planning_interface
        moveit_visual_tools
    DEPENDS
        EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
