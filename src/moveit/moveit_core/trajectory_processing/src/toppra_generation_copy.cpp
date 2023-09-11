//
// Created by spencer on 1/10/23.
//
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#include <moveit/trajectory_processing/toppra_generation.h>
#include <ros/console.h>
#include <Python.h>


namespace trajectory_processing
{
const std::string LOGNAME = "TOPPRA";
ToppraGeneration::ToppraGeneration(const double resample_dt, const double min_angle_change) : resample_dt_(resample_dt),
                                                                                        min_angle_change_(min_angle_change)
{
}

bool ToppraGeneration::runToppra(robot_trajectory::RobotTrajectory& trajectory, const double max_velocity_scaling_factor,
                              const double max_acceleration_scaling_factor, double init_path_velocity)
{
    if (trajectory.empty())
        return true;

    Py_Initialize();
    if (!Py_IsInitialized())
    {
        ROS_ERROR_NAMED(LOGNAME, "Python init failed.");
        return false;
    }

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/home/spencer/workspaces/dynamic_env_planning_ws/src/topp/toppra-python/toppra')")

    PyObject* pModule = PyImport_ImportModule("toppra_entrance");


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

    // Get the limits and create linear joint-space constraints
    toppra::Vector vel_limit_lower = Eigen::VectorXd::Zero(num_joints_);
    toppra::Vector vel_limit_upper = Eigen::VectorXd::Zero(num_joints_);
    toppra::Vector acc_limit_lower = Eigen::VectorXd::Zero(num_joints_);
    toppra::Vector acc_limit_upper = Eigen::VectorXd::Zero(num_joints_);

    toppra::Vector max_velocity{num_joints_};
    toppra::Vector max_acceleration{num_joints_};

    for (size_t j = 0; j < num_joints_; ++j)
    {
        const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars[j]);

        // Limits need to be non-zero, otherwise we never exit
        max_velocity[j] = 1.0;
        if (bounds.velocity_bounded_)
        {
            if (bounds.max_velocity_ < std::numeric_limits<double>::epsilon())
            {
                ROS_ERROR_NAMED(LOGNAME, "Invalid max_velocity %f specified for '%s', must be greater than 0.0",
                                bounds.max_velocity_, vars[j].c_str());
                return false;
            }
            max_velocity[j] =
                    std::min(std::fabs(bounds.max_velocity_), std::fabs(bounds.min_velocity_)) * velocity_scaling_factor;
            vel_limit_lower[j] = -max_velocity[j];
            vel_limit_upper[j] = max_velocity[j];
        }

        max_acceleration[j] = 1.0;
        if (bounds.acceleration_bounded_)
        {
            if (bounds.max_acceleration_ < std::numeric_limits<double>::epsilon())
            {
                ROS_ERROR_NAMED(LOGNAME, "Invalid max_acceleration %f specified for '%s', must be greater than 0.0",
                                bounds.max_acceleration_, vars[j].c_str());
                return false;
            }
            max_acceleration[j] = std::min(std::fabs(bounds.max_acceleration_), std::fabs(bounds.min_acceleration_)) *
                                  acceleration_scaling_factor;
            acc_limit_lower[j] = -max_acceleration[j];
            acc_limit_upper[j] = max_acceleration[j];
        }
    }

    toppra::LinearConstraintPtr ljv, lja;
    ljv = std::make_shared<toppra::constraint::LinearJointVelocity>(vel_limit_lower, vel_limit_upper);
    lja = std::make_shared<toppra::constraint::LinearJointAcceleration>(acc_limit_lower, acc_limit_upper);
    lja->discretizationType(toppra::DiscretizationType::Interpolation);
    toppra::LinearConstraintPtrs constraints{ljv, lja};

    // convert into Eigen data structs
    toppra::Vectors points(num_points_);
    fnum_points_ = 0;
    for (size_t p = 0; p < num_points_; ++p)
    {
        moveit::core::RobotStatePtr waypoint = trajectory.getWayPointPtr(p);
        toppra::Vector new_point{num_joints_};
        // The first point should always be kept
        bool diverse_point = (p == 0);

        for (size_t j = 0; j < num_joints_; ++j)
        {
            new_point[j] = waypoint->getVariablePosition(idx[j]);
            // If any joint angle is different, it's a unique waypoint
            if (p > 0 && std::fabs(new_point[j] - points[fnum_points_ - 1](j)) > min_angle_change_)
            {
                diverse_point = true;
            }
        }

        if (diverse_point)
        {
            points[fnum_points_] = new_point;
            fnum_points_++;
        }
    }

    // Return trajectory with only the first waypoint if there are not multiple diverse points
    if (fnum_points_ == 1)
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
    toppra::Vector path_positions = getPathPositions(points);

    Eigen::MatrixXd path_pos2_ = Eigen::MatrixXd::Zero(num_joints_, fnum_points_);
    formatVecToMat(points, path_pos2_);
    ROS_INFO_STREAM("points: \n" << path_pos2_ << std::endl);
    ROS_INFO_STREAM("path_positions: \n" << path_positions.transpose() << std::endl);
    ROS_INFO_STREAM("vel_limit_lower: \n" << vel_limit_lower.transpose() << std::endl);
    ROS_INFO_STREAM("vel_limit_upper: \n" << vel_limit_upper.transpose() << std::endl);
    ROS_INFO_STREAM("acc_limit_lower: \n" << acc_limit_lower.transpose() << std::endl);
    ROS_INFO_STREAM("acc_limit_upper: \n" << acc_limit_upper.transpose() << std::endl);
    ROS_INFO_NAMED(LOGNAME, "init_path_velocity: %f", init_path_velocity);

    // Cubicspline path
    toppra::BoundaryCond bc{"clamped"};
    toppra::BoundaryCondFull bc_type{bc, bc};
    toppra::PiecewisePolyPath cubic = toppra::PiecewisePolyPath::CubicSpline(points, path_positions, bc_type);
    toppra::GeometricPathPtr path;
    path = std::make_shared<toppra::PiecewisePolyPath>(cubic);

    // Now actually call the algorithm
    toppra::PathParametrizationAlgorithmPtr algo = std::make_shared<toppra::algorithm::TOPPRA>(constraints, path);
    toppra::ReturnCode rc1 = algo->computePathParametrization(init_path_velocity, 0);
    if (!(rc1 == toppra::ReturnCode::OK))
    {
        ROS_ERROR_NAMED(LOGNAME, "Unable to parameterize trajectory.");
        return false;
    }

    ROS_INFO_NAMED(LOGNAME, "Finish TOPPRA computation");

    toppra::ParametrizationData pd = algo->getParameterizationData();

    toppra::Vector gridpoints = pd.gridpoints;
    toppra::Vector vsquared = pd.parametrization;
    std::shared_ptr<toppra::parametrizer::ConstAccel> ca =
            std::make_shared<toppra::parametrizer::ConstAccel>(path, gridpoints, vsquared);
    Eigen::Matrix<toppra::value_type, 1, 2> interval;
    interval = ca->pathInterval();
    double duration = interval(1);

    // Compute sample count
    size_t sample_count = std::ceil(duration / resample_dt_);
    toppra::Vector sample_times = getSampleTimes(sample_count, duration);

    // Resample and fill in trajectory
    moveit::core::RobotState waypoint = moveit::core::RobotState(trajectory.getWayPoint(0));
    trajectory.clear();
    toppra::Vectors positions = ca->eval(sample_times, 0);
    toppra::Vectors velocities = ca->eval(sample_times, 1);
    toppra::Vectors accelerations = ca->eval(sample_times, 2);
    toppra::Vector path_velocities = ca->getPathVelocities();
    ROS_INFO_STREAM("path_velocities: \n" << path_velocities.transpose() << std::endl);
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

void ToppraGeneration::formatVecToMat(const std::vector<Eigen::VectorXd,
                                      Eigen::aligned_allocator<Eigen::VectorXd>>& vec, Eigen::MatrixXd& mat)
{
    mat.resize(vec.at(0).rows(), vec.size());
    for (size_t i = 0; i < vec.size(); i++)
    {
        mat.col(i) = vec.at(i);
    }
}

toppra::Vector ToppraGeneration::getSampleTimes(std::size_t sample_count, double duration)
{
    toppra::Vector times{sample_count + 1};
    for (size_t sample = 0; sample <= sample_count; ++sample)
    {
        double t = std::min(duration, sample * resample_dt_);
        times[sample] = t;
    }

    return times;
}

toppra::Vector ToppraGeneration::getPathPositions(toppra::Vectors& points)
{
    toppra::Vector path_positions{fnum_points_};
    path_positions[0] = 0.0;
    for (size_t p = 1; p < fnum_points_; ++p)
    {
        double dist = 0.0;
        for (std::size_t j = 0; j < num_joints_; ++j)
        {
            dist += std::pow(points[p](j) - points[p-1](j), 2);
        }
        dist /= num_joints_;
        path_positions[p] = path_positions[p - 1] + dist;
    }

    return path_positions;
}
}
