//
// Created by spencer on 1/10/23.
//

#ifndef DYNAMIC_ENV_PLANNING_WS_ADAPTER_STOPP_H
#define DYNAMIC_ENV_PLANNING_WS_ADAPTER_STOPP_H
#pragma once


#include <Eigen/Core>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ros/ros.h>
//#include <pybind11/pybind11.h>
//
//namespace py = pybind11;

namespace trajectory_processing
{
class StoppGeneration
{
public:
    StoppGeneration(const double resample_dt = 0.1, const double min_angle_change = 0.001);

    bool computeTrajectory(robot_trajectory::RobotTrajectory& trajectory,
                           const std::vector<double>& max_vel,
                           const std::vector<double>& max_acc,
                           const std::vector<double>& max_jerk,
                           const double max_velocity_scaling_factor = 1.0,
                           const double max_acceleration_scaling_factor = 1.0,
                           moveit::core::RobotStatePtr start_state_ptr = nullptr);
    void formatVecToMat(const std::vector<Eigen::VectorXd,
                        Eigen::aligned_allocator<Eigen::VectorXd>>& vec, Eigen::MatrixXd& mat);

private:

//    void pyListToVector1d(py::list pylist, std::vector<double>& vector1d);
//    void pyListToVector2d(py::list pylist, std::vector<std::vector<double>>& vector2d);
    std::vector<double> getPathPositions(std::vector<std::vector<double>>& points);
    std::vector<double> getSampleTimes(std::size_t sample_count, double duration);

    bool stoppAlgorithm(std::vector<std::vector<double>>& points, std::vector<double>& path_positions,
                        std::vector<std::vector<double>>& vel_lim, std::vector<std::vector<double>>& acc_lim,
                        std::vector<std::vector<double>>& jerk_lim, double resample_dt, double init_path_velocity=0,
                        std::vector<double> init_qs = {0});

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

    /*
    void setSampleRange(std::vector<double>& path_velocities)
    {
      int path_len = path_velocities.size();
      sample_range_.reserve(path_len);
      for (size_t i = 0; i < path_len; ++i)
      {
        if (i == 0)
          std::vector<double> range = {path_velocities[i], path_velocities[i]};
        else
          std::vector<double> range = {0.0, path_velocities[i]};

        sample_range_.push_back(range);
      }
    }
*/

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
    std::vector<std::vector<double>> sample_range_;
    std::vector<std::vector<double>> positions_;
    std::vector<std::vector<double>> velocities_;
    std::vector<std::vector<double>> accelerations_;
    std::vector<double> path_velocities_;

};
}


#endif //DYNAMIC_ENV_PLANNING_WS_ADAPTER_STOPP_H
