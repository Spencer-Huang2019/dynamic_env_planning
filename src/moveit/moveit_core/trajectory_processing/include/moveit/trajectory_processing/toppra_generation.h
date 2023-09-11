//
// Created by spencer on 1/10/23.
//

#ifndef DYNAMIC_ENV_PLANNING_WS_ADAPTER_TOPPRA_H
#define DYNAMIC_ENV_PLANNING_WS_ADAPTER_TOPPRA_H
#pragma once


#include <Eigen/Core>
#include <moveit/robot_trajectory/robot_trajectory.h>
//#include <toppra/algorithm/toppra.hpp>
//#include <toppra/toppra.hpp>
#include <ros/ros.h>


namespace trajectory_processing
{
class ToppraGeneration
{
public:
    ToppraGeneration(const double resample_dt = 0.1, const double min_angle_change = 0.001);
//    bool computeTrajectory(robot_trajectory::RobotTrajectory& trajectory, const double max_velocity_scaling_factor = 1.0,
//                   const double max_acceleration_scaling_factor = 1.0, double init_path_velocity = 0.0);
    bool computeTrajectory(robot_trajectory::RobotTrajectory& trajectory,
                           const double max_velocity_scaling_factor = 1.0,
                           const double max_acceleration_scaling_factor = 1.0,
                           moveit::core::RobotStatePtr start_state_ptr = nullptr);
    void formatVecToMat(const std::vector<Eigen::VectorXd,
                        Eigen::aligned_allocator<Eigen::VectorXd>>& vec, Eigen::MatrixXd& mat);

private:
    std::vector<double> getPathPositions(std::vector<std::vector<double>>& points);
    std::vector<double> getSampleTimes(std::size_t sample_count, double duration);
//    void pyListToVector1d(pybind11::list pylist, std::vector<double>& vector1d);
//    void pyListToVector2d(pybind11::list pylist, std::vector<std::vector<double>>& vector2d);

    bool callToppraPython(std::vector<std::vector<double>>& points, std::vector<double>& path_positions,
                          std::vector<std::vector<double>>& vel_lim, std::vector<std::vector<double>>& acc_lim,
                          double resample_dt, double init_path_velocity=0, std::vector<double> init_qs = {0});

    void setSampleTimes(std::vector<double>& sample_times)
    {
        sample_times_ = sample_times;
    }

    std::vector<double> getSampleTimes()
    {
        return sample_times_;
    }

    void setPositions(std::vector<std::vector<double>>& positions)
    {
        positions_ = positions;
    }

    std::vector<std::vector<double>> getPositions()
    {
        return positions_;
    }

    void setVelocities(std::vector<std::vector<double>>& velocities)
    {
        velocities_ = velocities;
    }

    std::vector<std::vector<double>> getVelocities()
    {
        return velocities_;
    }

    void setAccelerations(std::vector<std::vector<double>>& accelerations)
    {
        accelerations_ = accelerations;
    }

    std::vector<std::vector<double>> getAccelerations()
    {
        return accelerations_;
    }

    void setPathVelocities(std::vector<double>& path_velocities)
    {
        path_velocities_ = path_velocities;
    }

    std::vector<double> getPathVelocities()
    {
        return path_velocities_;
    }

    const double resample_dt_;
    const double min_angle_change_;
    int num_joints_;
    int num_points_;
    size_t fnum_points_;
    bool already_init_{false};

    std::vector<double> sample_times_;
    std::vector<std::vector<double>> positions_;
    std::vector<std::vector<double>> velocities_;
    std::vector<std::vector<double>> accelerations_;
    std::vector<double> path_velocities_;

};
}


#endif //DYNAMIC_ENV_PLANNING_WS_ADAPTER_TOPPRA_H
