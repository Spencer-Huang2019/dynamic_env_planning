cmake_minimum_required(VERSION 3.0.2)

project(file_operation_tools)

set(CMAKE_CXX_STANDARD 14)
set(FILE_OPT_TOOLS_LIBS file_operation_tools_library)
set(YAML_CPP_LIBS /home/spencer/workspaces/dynamic_env_planning/src/move_dynamic/tools/yaml-cpp/build/libyaml-cpp.so)

find_package(catkin REQUIRED COMPONENTS
        moveit_msgs
        moveit_core
#        yaml_cpp
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${FILE_OPT_TOOLS_LIBS}
)

include_directories(include /home/spencer/workspaces/dynamic_env_planning/src/move_dynamic/tools/yaml-cpp/include)
include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
        )

add_library(${FILE_OPT_TOOLS_LIBS}
        src/write_trajectory.cpp
        )
set_target_properties(${FILE_OPT_TOOLS_LIBS} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${FILE_OPT_TOOLS_LIBS} ${YAML_CPP_LIBS} ${catkin_LIBRARIES})

#add_dependencies(${FILE_OPT_TOOLS_LIBS} ${catkin_EXPORTED_TARGETS})
install(TARGETS ${FILE_OPT_TOOLS_LIBS}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})