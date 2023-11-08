cmake_minimum_required(VERSION 3.1.3)
project(move_dynamic_trajectory_adapter)

set(MOVE_DYNAMIC_LIB_NAME move_dynamic_planning_request_adapter_plugins)
set(SOURCE_FILE src/adapter_toppra.cpp)

find_package(Boost REQUIRED system filesystem date_time program_options thread chrono)
find_package(catkin REQUIRED COMPONENTS
        moveit_core
        )

find_package(Eigen3 REQUIRED)

add_library(${MOVE_DYNAMIC_LIB_NAME} ${SOURCE_FILE})
set_target_properties(${MOVE_DYNAMIC_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set(TOPPRA_LIBS /home/spencer/workspaces/dynamic_env_planning/build/toppra/src/libtoppra.so)
target_link_libraries(${MOVE_DYNAMIC_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TOPPRA_LIBS})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${MOVE_DYNAMIC_LIB_NAME}

)

include_directories(include /home/spencer/workspaces/dynamic_env_planning/devel/include)
include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        )

install(TARGETS ${MOVE_DYNAMIC_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
