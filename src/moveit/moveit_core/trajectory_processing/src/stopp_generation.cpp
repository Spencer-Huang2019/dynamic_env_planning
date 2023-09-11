//
// Created by spencer on 1/10/23.
//
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/stopp_generation.h>
#include <ros/console.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>


namespace py = pybind11;


void pyListToVector1d(py::list pylist, std::vector<double>& vector1d)
{
  for (const auto ele : pylist)
  {
    vector1d.push_back(ele.cast<double>());
  }
}

void pyListToVector2d(py::list pylist, std::vector<std::vector<double>>& vector2d)
{
  for (const auto ele1 : pylist)
  {
    std::vector<double> temp;
    temp.reserve(7);
    for (const auto ele2 : ele1)
    {
      temp.push_back(ele2.cast<double>());
    }
    vector2d.push_back(temp);
  }
}

namespace trajectory_processing
{

const std::string LOGNAME = "STOPP";
StoppGeneration::StoppGeneration(const double resample_dt, const double min_angle_change) : resample_dt_(resample_dt),
                                                                                        min_angle_change_(min_angle_change)
{
}

bool StoppGeneration::computeTrajectory(robot_trajectory::RobotTrajectory& trajectory,
                                        const std::vector<double>& max_vel,
                                        const std::vector<double>& max_acc,
                                        const std::vector<double>& max_jerk,
                                        const double max_velocity_scaling_factor,
                                        const double max_acceleration_scaling_factor,
                                        moveit::core::RobotStatePtr start_state_ptr)
{
    if (trajectory.empty())
        return true;

    const moveit::core::JointModelGroup* group = trajectory.getGroup();
    if (!group)
    {
        ROS_ERROR_NAMED(LOGNAME, "It looks like the planner did not set the group the plan was computed for");
        return false;
    }

    // Validate scaling
    double velocity_scaling_factor = 1.0;
    if (max_velocity_scaling_factor > 0.0 && max_velocity_scaling_factor <= 1.0)
    {
        velocity_scaling_factor = max_velocity_scaling_factor;
    }
    else if (max_velocity_scaling_factor == 0.0)
    {
        ROS_DEBUG_NAMED(LOGNAME, "A max_velocity_scaling_factor of 0.0 was specified, defaulting to %f instead.",
                        velocity_scaling_factor);
    }
    else
    {
        ROS_WARN_NAMED(LOGNAME, "Invalid max_velocity_scaling_factor %f specified, defaulting to %f instead.",
                       max_velocity_scaling_factor, velocity_scaling_factor);
    }

    double acceleration_scaling_factor = 1.0;
    if (max_acceleration_scaling_factor > 0.0 && max_acceleration_scaling_factor <= 1.0)
    {
        acceleration_scaling_factor = max_acceleration_scaling_factor;
    }
    else if (max_acceleration_scaling_factor == 0.0)
    {
        ROS_DEBUG_NAMED(LOGNAME, "A max_acceleration_scaling_factor of 0.0 was specified, defaulting to %f instead.",
                        acceleration_scaling_factor);
    }
    else
    {
        ROS_WARN_NAMED(LOGNAME, "Invalid max_acceleration_scaling_factor %f specified, defaulting to %f instead.",
                       max_acceleration_scaling_factor, acceleration_scaling_factor);
    }

    // This lib does not actually work properly when angles wrap around, so we need to unwind the path first
    trajectory.unwind();

    // This is pretty much copied from IterativeParabolicTimeParameterization::applyVelocityConstraints
    const std::vector<std::string>& vars = group->getVariableNames();
    const std::vector<int>& idx = group->getVariableIndexList();
    const moveit::core::RobotModel& rmodel = group->getParentModel();
    num_joints_ = group->getVariableCount();
    num_points_ = trajectory.getWayPointCount();

    // Get the limits
    std::vector<std::vector<double>> vel_lim(num_joints_);
    std::vector<std::vector<double>> acc_lim(num_joints_);
    std::vector<std::vector<double>> jerk_lim(num_joints_);
    bool reset_max_vel = max_vel.empty() ? false : true;
    bool reset_max_acc = max_acc.empty() ? false : true;
    bool reset_max_jerk = max_jerk.empty() ? false : true;

    for (size_t j = 0; j < num_joints_; ++j)
    {
        const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars[j]);

        // Limits need to be non-zero, otherwise we never exit
        if (reset_max_vel)
        {
          if (max_vel[j] < std::numeric_limits<double>::epsilon())
          {
            ROS_ERROR_NAMED(LOGNAME, "Invalid max_velocity %f specified for '%s', must be greater than 0.0",
                            max_vel[j], vars[j].c_str());
            return false;
          }
          double max_velocity = max_vel[j] * velocity_scaling_factor;
          vel_lim[j] = {-max_velocity, max_velocity};
        }
        else if (bounds.velocity_bounded_)
        {
            if (bounds.max_velocity_ < std::numeric_limits<double>::epsilon())
            {
                ROS_ERROR_NAMED(LOGNAME, "Invalid max_velocity %f specified for '%s', must be greater than 0.0",
                                bounds.max_velocity_, vars[j].c_str());
                return false;
            }
            double max_velocity =
                    std::min(std::fabs(bounds.max_velocity_), std::fabs(bounds.min_velocity_)) * velocity_scaling_factor;
            vel_lim[j] = {-max_velocity, max_velocity};
        }

      if (reset_max_acc)
      {
        if (max_acc[j] < std::numeric_limits<double>::epsilon())
        {
          ROS_ERROR_NAMED(LOGNAME, "Invalid max_velocity %f specified for '%s', must be greater than 0.0",
                          max_acc[j], vars[j].c_str());
          return false;
        }
        double max_acceleration = max_acc[j] * acceleration_scaling_factor;
        acc_lim[j] = {-max_acceleration, max_acceleration};
      }
      else if (bounds.acceleration_bounded_)
      {
          if (bounds.max_acceleration_ < std::numeric_limits<double>::epsilon())
          {
              ROS_ERROR_NAMED(LOGNAME, "Invalid max_acceleration %f specified for '%s', must be greater than 0.0",
                              bounds.max_acceleration_, vars[j].c_str());
              return false;
          }
          double max_acceleration = std::min(std::fabs(bounds.max_acceleration_), std::fabs(bounds.min_acceleration_)) *
                                    acceleration_scaling_factor;
          acc_lim[j] = {-max_acceleration, max_acceleration};
      }

      if (reset_max_jerk)
      {
        if (max_jerk[j] < std::numeric_limits<double>::epsilon())
        {
          ROS_ERROR_NAMED(LOGNAME, "Invalid max_velocity %f specified for '%s', must be greater than 0.0",
                          max_jerk[j], vars[j].c_str());
          return false;
        }
        jerk_lim[j] = {-max_jerk[j], max_jerk[j]};
      }
      else if (bounds.jerk_bounded_)
      {
        if (bounds.max_jerk_ < std::numeric_limits<double>::epsilon())
        {
          ROS_ERROR_NAMED(LOGNAME, "Invalid max_jerk %f specified for '%s', must be greater than 0.0",
                          bounds.max_jerk_, vars[j].c_str());
          return false;
        }
        double max_jerk = std::min(std::fabs(bounds.max_jerk_), std::fabs(bounds.min_jerk_)) *
                                  acceleration_scaling_factor;
        jerk_lim[j] = {-max_jerk, max_jerk};
      }
    }

    // convert waypoints into vector
    std::vector<std::vector<double>> points;
    for (size_t p = 0; p < num_points_; ++p)
    {
        moveit::core::RobotStatePtr waypoint = trajectory.getWayPointPtr(p);
        std::vector<double> new_point(num_joints_);
        // The first point should always be kept
        bool diverse_point = (p == 0);

        for (size_t j = 0; j < num_joints_; ++j)
        {
            new_point[j] = waypoint->getVariablePosition(idx[j]);
            // If any joint angle is different, it's a unique waypoint
            if (p > 0 && std::fabs(new_point[j] - points.back()[j]) > min_angle_change_)
            {
                diverse_point = true;
            }
        }

        if (diverse_point)
            points.push_back(new_point);
    }

    // Return trajectory with only the first waypoint if there are not multiple diverse points
    if (points.size() == 1)
    {
        ROS_DEBUG_NAMED(LOGNAME,
                        "Trajectory is parameterized with 0.0 dynamics since it only contains a single distinct waypoint.");
        moveit::core::RobotState waypoint = moveit::core::RobotState(trajectory.getWayPoint(0));
        waypoint.zeroVelocities();
        waypoint.zeroAccelerations();
        trajectory.clear();
        trajectory.addSuffixWayPoint(waypoint, 0.0);
        return true;
    }

    // get path_positions for points
    std::vector<double> path_positions = getPathPositions(points);

    if (start_state_ptr)
    {
        double init_path_velocity = start_state_ptr->getVariablePathVelocity();
        std::vector<double> init_qs;
        init_qs.reserve(7);
        auto init_velocity = start_state_ptr->getVariableVelocities();
        for (std::size_t i = 0; i < 7; ++i)
        {
            init_qs.push_back(init_velocity[i] / init_path_velocity);
        }

        if (!stoppAlgorithm(points, path_positions, vel_lim, acc_lim, jerk_lim, resample_dt_, init_path_velocity, init_qs))
        {
            return false;
        }
    }
    else
    {
        if (!stoppAlgorithm(points, path_positions, vel_lim, acc_lim, jerk_lim, resample_dt_))
        {
            return false;
        }
    }

    std::vector<double> sample_times = getSampleTimes();
    std::vector<std::vector<double>> positions = getPositions();
    std::vector<std::vector<double>> velocities = getVelocities();
    std::vector<std::vector<double>> accelerations = getAccelerations();
    std::vector<double> path_velocities = getPathVelocities();

    // Resample and fill in trajectory
    moveit::core::RobotState waypoint = moveit::core::RobotState(trajectory.getWayPoint(0));
    trajectory.clear();
    double last_t = 0;
    for (size_t i = 0; i < sample_times.size(); ++i)
    {
        for (size_t j = 0; j < num_joints_; ++j)
        {
            waypoint.setVariablePosition(idx[j], positions[i][j]);
            waypoint.setVariableVelocity(idx[j], velocities[i][j]);
            waypoint.setVariableAcceleration(idx[j], accelerations[i][j]);
        }
        waypoint.setVariablePathVelocity(path_velocities[i]);

        trajectory.addSuffixWayPoint(waypoint, sample_times[i] - last_t);
        last_t = sample_times[i];
    }
    return true;
}

bool StoppGeneration::stoppAlgorithm(std::vector<std::vector<double>>& points, std::vector<double>& path_positions,
                                     std::vector<std::vector<double>>& vel_lim, std::vector<std::vector<double>>& acc_lim,
                                     std::vector<std::vector<double>>& jerk_lim,double resample_dt,
                                     double init_path_velocity, std::vector<double> init_qs)
{
    ROS_INFO_NAMED(LOGNAME, "run STOPP");

//    py::scoped_interpreter python;

    py::module sys = py::module::import("sys");
    sys.attr("path").attr("append")("/home/spencer/workspaces/dynamic_env_planning_ws/src/topp/stopp/toppra");
    py::module stopp = py::module::import("stopp_entrance");

    py::object result = stopp.attr("call_stopp")(points,
            path_positions, vel_lim, acc_lim, jerk_lim, resample_dt, init_path_velocity, init_qs);
    py::list result_list = result.cast<py::list>();
    ROS_INFO_NAMED(LOGNAME, "STOPP finish computation");

    std::vector<double> sample_times;
    pyListToVector1d(result_list[0], sample_times);
    fnum_points_ = sample_times.size();

    std::vector<std::vector<double>> positions;
    positions.reserve(fnum_points_);
    pyListToVector2d(result_list[1], positions);

    std::vector<std::vector<double>> velocities;
    velocities.reserve(fnum_points_);
    pyListToVector2d(result_list[2], velocities);

    std::vector<std::vector<double>> accelerations;
    accelerations.reserve(fnum_points_);
    pyListToVector2d(result_list[3], accelerations);

    std::vector<double> path_velocity;
    path_velocity.reserve(fnum_points_);
    pyListToVector1d(result_list[4], path_velocity);

    setSampleTimes(sample_times);
    setPositions(positions);
    setVelocities(velocities);
    setAccelerations(accelerations);
    setPathVelocities(path_velocity);

    return true;
}

std::vector<double> StoppGeneration::getSampleTimes(std::size_t sample_count, double duration)
{
    std::vector<double> times(sample_count + 1);
    for (size_t sample = 0; sample <= sample_count; ++sample)
    {
        double t = std::min(duration, sample * resample_dt_);
        times[sample] = t;
    }

    return times;
}

std::vector<double> StoppGeneration::getPathPositions(std::vector<std::vector<double>>& points)
{
    std::size_t num_point = points.size();
    std::vector<double> path_positions(num_point);
    path_positions[0] = 0.0;
    for (size_t p = 1; p < num_point; ++p)
    {
        double dist = 0.0;
        for (std::size_t j = 0; j < num_joints_; ++j)
        {
            dist += std::pow(points[p][j] - points[p-1][j], 2);
        }
        dist /= num_joints_;
        path_positions[p] = path_positions[p - 1] + dist;
    }

    return path_positions;
}
}
