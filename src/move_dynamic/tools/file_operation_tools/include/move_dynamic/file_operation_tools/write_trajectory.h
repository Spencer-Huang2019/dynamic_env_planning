//
// Created by spencer on 1/7/23.
//

#ifndef DYNAMIC_ENV_PLANNING_WS_WRITE_TRAJECTORY_YAML_H
#define DYNAMIC_ENV_PLANNING_WS_WRITE_TRAJECTORY_YAML_H

#pragma once

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "yaml-cpp/yaml.h"


namespace move_dynamic
{
class WriteTrajectory
{
public:
    WriteTrajectory();
    ~WriteTrajectory();

    void writeTrajectoryMsg2Yaml(const moveit_msgs::RobotTrajectory& trajectory, const std::string& filename);
    void writeTrajectoryMsg2Yaml(robot_trajectory::RobotTrajectoryPtr& trajectory, const std::string& filename);
    void writeTrajectoryMsg2Yaml(const std::vector<moveit_msgs::RobotTrajectory>& trajectories, const std::string& filename);
//private:
//    YAML::Node config_;
};
}

#endif //DYNAMIC_ENV_PLANNING_WS_WRITE_TRAJECTORY_YAML_H
