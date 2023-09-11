//
// Created by spencer on 1/10/23.
//

#ifndef DYNAMIC_ENV_PLANNING_WS_ADAPTER_TOPPRA_H
#define DYNAMIC_ENV_PLANNING_WS_ADAPTER_TOPPRA_H
#pragma once


#include <Eigen/Core>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/toppra.hpp>
#include <ros/ros.h>

namespace move_dynamic
{
namespace trajectory_adapter
{
class AdapterToppra
{
public:
    AdapterToppra(const double resample_dt = 0.1, const double min_angle_change = 0.001);
    bool runToppra(robot_trajectory::RobotTrajectory& trajectory, const double max_velocity_scaling_factor = 1.0,
                   const double max_acceleration_scaling_factor = 1.0, double init_path_velocity = 0.0);
    toppra::Vector getPathPositions(toppra::Vectors& points);
    toppra::Vector getSampleTimes(std::size_t sample_count, double duration);

private:
    const double resample_dt_;
    const double min_angle_change_;
    int num_joints_;
    int num_points_;
};
}
}

#endif //DYNAMIC_ENV_PLANNING_WS_ADAPTER_TOPPRA_H
