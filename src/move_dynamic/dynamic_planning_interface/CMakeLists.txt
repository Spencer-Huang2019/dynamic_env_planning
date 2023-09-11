cmake_minimum_required(VERSION 3.0.2)
project(move_dynamic_planning_interface)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        pluginlib
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        roscpp
        std_msgs
        moveit_msgs
        trajectory_msgs
        move_dynamic_msgs
        rospy
        )

find_package(PythonInterp REQUIRED)
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" REQUIRED)

find_package(Boost REQUIRED)
if(Boost_VERSION LESS 106700)
    set(BOOST_PYTHON_COMPONENT python)
else()
    set(BOOST_PYTHON_COMPONENT python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
endif()

find_package(Boost REQUIRED COMPONENTS
        date_time
        filesystem
        program_options
        ${BOOST_PYTHON_COMPONENT}
        system
        thread
        )

find_package(eigenpy REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
    py_bindings_tools/include
    move_dynamic_interface/include
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
        ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES
        move_dynamic_interface
        move_dynamic_py_bindings_tools
  CATKIN_DEPENDS
        actionlib
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        roscpp
        std_msgs
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_subdirectory(py_bindings_tools)
add_subdirectory(move_dynamic_interface)