cmake_minimum_required(VERSION 3.0.2)
project(move_dynamic_move_group)

find_package(Boost REQUIRED system filesystem date_time program_options thread)
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
        file_operation_tools
        )

set(CAPABILITIES_BASE move_dynamic_move_group_capabilities_base)
set(DEFAULT_CAPABILITIES move_dynamic_move_group_default_capabilities)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${CAPABILITIES_BASE}
        CATKIN_DEPENDS actionlib moveit_core moveit_ros_planning moveit_ros_planning_interface roscpp std_msgs
)

include_directories(include)
include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        )

add_library(${CAPABILITIES_BASE} src/move_dynamic_capability.cpp)
set_target_properties(${CAPABILITIES_BASE} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
add_dependencies(${CAPABILITIES_BASE} ${catkin_EXPORTED_TARGETS})

add_executable(move_dynamic src/move_dynamic.cpp)

add_library(${DEFAULT_CAPABILITIES}
        src/execute_wpts_capability.cpp
        src/planning_service_capability.cpp)
set_target_properties(${DEFAULT_CAPABILITIES} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
add_dependencies(${DEFAULT_CAPABILITIES} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${CAPABILITIES_BASE} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(move_dynamic ${CAPABILITIES_BASE} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(${DEFAULT_CAPABILITIES} ${CAPABILITIES_BASE} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${CAPABILITIES_BASE} ${DEFAULT_CAPABILITIES}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES
        md_default_capabilities_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})