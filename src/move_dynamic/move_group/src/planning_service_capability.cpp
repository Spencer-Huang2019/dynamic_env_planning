//
// Created by spencer on 12/13/22.
//

#include "move_dynamic/move_group/planning_service_capability.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "move_dynamic/move_group/capability_names.h"
#include "move_dynamic/file_operation_tools/write_trajectory.h"

constexpr char LOGNAME[] = "Planning_Capability";

move_dynamic::WriteTrajectory write_file;
std::string dir1 = "/home/spencer/workspaces/dynamic_env_planning_ws/src/franka_move/files/global_traj/";
std::string dir2 = "/home/spencer/workspaces/dynamic_env_planning_ws/src/franka_move/files/local_traj/";
std::string dir3 = "/home/spencer/workspaces/dynamic_env_planning_ws/src/franka_move/files/combine_traj/";

namespace move_group
{
MoveDynamicPlanningService::MoveDynamicPlanningService()
: MoveDynamicCapability("PlanningService"), move_group_interface_(group_name_)
{
}

void MoveDynamicPlanningService::initialize() {
    planning_action_server_ = std::make_unique < actionlib::SimpleActionServer < move_dynamic_msgs::PlanningAction >> (
            root_node_handle_, PLANNING_SERVICE_NAME, [this](const auto &goal) { executePlanningCallback(goal); }, false);
    planning_action_server_->start();

}

void MoveDynamicPlanningService::executePlanningCallback(const move_dynamic_msgs::PlanningGoalConstPtr &goal)
{
  move_dynamic_msgs::PlanningResult action_res;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Setting the parameters
  root_node_handle_.setParam("move_group/planning_pipelines/ompl/resample_dt", goal->resample_dt);
  move_group_interface_.setPlannerId(goal->planner_id);
  move_group_interface_.setGoalJointTolerance(0.001);
  move_group_interface_.setMaxVelocityScalingFactor(goal->scaling_factor);
  move_group_interface_.setMaxAccelerationScalingFactor(goal->scaling_factor);

  if (!goal->max_vel.empty())
    move_group_interface_.setMaxVel(goal->max_vel);
  if (!goal->max_acc.empty())
    move_group_interface_.setMaxAcc(goal->max_acc);
  if (!goal->max_jerk.empty())
    move_group_interface_.setMaxJerk(goal->max_jerk);

  // add obstacles
  size_t n_obs = goal->obstacles.size();
  ROS_INFO_NAMED(LOGNAME, "obstacle nums = %d", n_obs);
  if (n_obs > 0)
  {
    std::vector<moveit_msgs::CollisionObject> collision_objects(n_obs);
    for (size_t i = 0; i < n_obs; ++i)
    {
      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = move_group_interface_.getPlanningFrame();

      // The id of the object is used to identify it.
      collision_object.id = std::to_string(i);

      auto goal_obstacle = goal->obstacles[i];
      collision_object.primitives.push_back(goal_obstacle.primitive);
      collision_object.primitive_poses.push_back(goal_obstacle.pose);
      collision_object.operation = collision_object.ADD;

      collision_objects[i] = collision_object;
    }
    planning_scene_interface.addCollisionObjects(collision_objects);
    ROS_INFO_NAMED(LOGNAME, "Finish adding obstacle");
  }

  const moveit::core::JointModelGroup *joint_model_group =
          move_group_interface_.getCurrentState()->getJointModelGroup(group_name_);

  is_connected_ = connectServer();

  if (goal->replanning)
  {
    executeLocalPlanning(goal, action_res);
  }
  else
  {
    executeGlobalPlanning(goal, action_res);
  }
}

void MoveDynamicPlanningService::executeGlobalPlanning(const move_dynamic_msgs::PlanningGoalConstPtr &goal,
                                                       move_dynamic_msgs::PlanningResult& action_res)
{
    ROS_WARN_NAMED(LOGNAME, "Start global planning");
    if (!goal->start_state.joint_state.name.empty())
        move_group_interface_.setStartState(goal->start_state);
    else
        move_group_interface_.setStartStateToCurrentState();
    move_group_interface_.setJointValueTarget(goal->joint_goal);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED(LOGNAME, "Global planning with %s result", success ? "SUCCEED" : "FAILED");
    if (success)
    {
        action_res.error_code = true;
        planning_action_server_->setSucceeded(action_res);
        global_trajectory_ = my_plan.trajectory_;
        trajectories_.clear();
        breakTrajectory(true);
        ROS_INFO_STREAM("Breaking trajectory size in global planning: " << global_trajectory_.joint_trajectory.points.size() - 1);
//        ROS_INFO_STREAM(global_trajectory_);
        sendStartExeWptsSignal();
    }
    else
    {
        action_res.error_code = false;
        planning_action_server_->setAborted(action_res);
    }
}

void MoveDynamicPlanningService::executeLocalPlanning(const move_dynamic_msgs::PlanningGoalConstPtr &goal,
                                                      move_dynamic_msgs::PlanningResult& action_res)
{

    ROS_WARN_NAMED(LOGNAME, "Start local replanning.");
    action_res.error_code = true;
    planning_action_server_->setSucceeded(action_res);

    std::string filename1 = dir1 + "global_traj.yaml";
    std::string filename2 = dir2 + "local_traj.yaml";
    std::string filename3 = dir3 + "combine_traj.yaml";
    write_file.writeTrajectoryMsg2Yaml(global_trajectory_, filename1);

    // setting the start states
    std::vector<int> start_indexes = determineStartStates();
    std::vector<moveit_msgs::RobotState> start_states;
    start_states.resize(start_indexes.size());
    std::vector<std::string> JOINT_NAMES = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                                            "panda_joint5", "panda_joint6", "panda_joint7"};

    for (std::size_t i = 0; i < start_indexes.size(); ++i)
    {
        start_states[i].joint_state.name = JOINT_NAMES;
        start_states[i].joint_state.position = global_trajectory_.joint_trajectory.points[start_indexes[i]].positions;
        start_states[i].joint_state.velocity = global_trajectory_.joint_trajectory.points[start_indexes[i]].velocities;
        start_states[i].path_velocity = global_trajectory_.joint_trajectory.points[start_indexes[i]].effort[0];
    }

    move_group_interface_.setStartStates(start_states);
    move_group_interface_.setJointValueTarget(goal->joint_goal);
    move_group_interface_.setStartIndexes(start_indexes);

    moveit::planning_interface::MoveGroupInterface::PlanMulti my_plan;

    bool success = move_group_interface_.planMulti(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO_NAMED(LOGNAME, "Local planning with %s result", success ? "SUCCEED" : "FAILED");
    if (success)
    {
        int start_index = start_indexes[my_plan.start_index_];
        local_trajectory_ = my_plan.trajectories_[my_plan.start_index_];
        write_file.writeTrajectoryMsg2Yaml(local_trajectory_, filename2);
        local_trajectories_.clear();
        breakTrajectory(false);
        std::string filename4 = dir2 + "local_trajs.yaml";
        write_file.writeTrajectoryMsg2Yaml(local_trajectories_, filename4);
        int execution_index = -1;
        root_node_handle_.getParam("move_dynamic_interface/execute/execution_index", execution_index);
        ROS_INFO_STREAM("execution_index: " << execution_index);
        if (start_index > execution_index && execution_index != -1)
            updatePlan(start_index);
        else
            ROS_WARN_NAMED(LOGNAME, "Updating original plan fails.");
        write_file.writeTrajectoryMsg2Yaml(trajectories_, filename3);
    }
}

void MoveDynamicPlanningService::updatePlan(const int index)
{
    trajectories_.erase(trajectories_.begin() + index, trajectories_.end());
    trajectories_.insert(trajectories_.end(), local_trajectories_.begin(), local_trajectories_.end());
    ROS_INFO_STREAM("Breaking trajectory size in after updating: " << trajectories_.size());
}

std::vector<int> MoveDynamicPlanningService::determineStartStates()
{
    int num_waypoint = global_trajectory_.joint_trajectory.points.size();
//    ROS_INFO_STREAM("trajectory size = " << num_waypoint);
    int mid = 2 * num_waypoint / 3;
    std::vector<int> indexes = {mid + 1, mid, mid - 1};
    ROS_INFO_STREAM("indexes[0] = " << indexes[0]);
    return indexes;
}

void MoveDynamicPlanningService::breakTrajectory(bool global)
{
//    ROS_INFO_STREAM("origin trajectory" << trajectory_);
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    std::string frame_id;
    std::vector<std::string> jointNames;

    if (global)
    {
        points = global_trajectory_.joint_trajectory.points;
        frame_id = global_trajectory_.joint_trajectory.header.frame_id;
        jointNames = global_trajectory_.joint_trajectory.joint_names;
        trajectories_.reserve(points.size() - 1);
    }
    else
    {
        points = local_trajectory_.joint_trajectory.points;
        frame_id = local_trajectory_.joint_trajectory.header.frame_id;
        jointNames = local_trajectory_.joint_trajectory.joint_names;
        local_trajectories_.reserve(points.size() - 1);
    }

    for (std::size_t i = 0; i < points.size() - 1; ++i)
    {
        moveit_msgs::RobotTrajectory robotTraj;
        robotTraj.joint_trajectory.joint_names = jointNames;
        robotTraj.joint_trajectory.header.frame_id = frame_id;
        robotTraj.joint_trajectory.header.seq = i;
        robotTraj.joint_trajectory.header.stamp = ros::Time(0);

        ros::Duration duration = points[i].time_from_start;
        ros::Duration duration_p1 = points[i + 1].time_from_start;
        robotTraj.joint_trajectory.points.resize(2);
        for (std::size_t k = 0; k < 2; ++k)
        {
            if (k == 0)
            {
                robotTraj.joint_trajectory.points[k].time_from_start = ros::Duration(0);
            }
            else
            {
                robotTraj.joint_trajectory.points[k].time_from_start = duration_p1 - duration;
            }

            robotTraj.joint_trajectory.points[k].positions.resize(7);
            robotTraj.joint_trajectory.points[k].velocities.resize(7);
            robotTraj.joint_trajectory.points[k].accelerations.resize(7);
            robotTraj.joint_trajectory.points[k].positions = points[k + i].positions;
            robotTraj.joint_trajectory.points[k].velocities = points[k + i].velocities;
            robotTraj.joint_trajectory.points[k].accelerations = points[k + i].accelerations;
        }
        if (global)
            trajectories_.push_back(robotTraj);
        else
            local_trajectories_.push_back(robotTraj);
    }
}

bool MoveDynamicPlanningService::connectServer()
{
    std::string name = EXECUTE_WPTS_NAME;

    ros::WallDuration wait_for_servers = ros::WallDuration(2);
    ros::WallTime timeout_for_servers = ros::WallTime::now() + wait_for_servers;
    if (wait_for_servers == ros::WallDuration())
        timeout_for_servers = ros::WallTime();  // wait for ever
    double allotted_time = wait_for_servers.toSec();

    exe_wpts_action_client_ =
            std::make_unique < actionlib::SimpleActionClient <move_dynamic_msgs::StartExeAction>> (
                    root_node_handle_, name, false);
    if(waitForAction(exe_wpts_action_client_, name, timeout_for_servers, allotted_time))
    {
        return true;
    }
    else
    {
        return false;
    }

}

void MoveDynamicPlanningService::sendStartExeWptsSignal() {
    while (!is_connected_)
    {
        is_connected_ = connectServer();
    }
    move_dynamic_msgs::StartExeGoal goal;
    exe_wpts_action_client_->sendGoal(goal);
    exe_wpts_action_client_->waitForResult(ros::Duration(3.0));
}

template <typename T>
bool MoveDynamicPlanningService::waitForAction(const T &action, const std::string &name, const ros::WallTime &timeout,
                                                        double allotted_time) const
{
    ROS_DEBUG_NAMED(LOGNAME, "Waiting for action server (%s)...", name.c_str());

    // wait for the server (and spin as needed)
    if (timeout == ros::WallTime())  // wait forever
    {
        while (root_node_handle_.ok() && !action->isServerConnected()) {
            ros::WallDuration(0.001).sleep();
            // explicit ros::spinOnce on the callback queue used by NodeHandle that manages the action client
            ros::CallbackQueue *queue = dynamic_cast<ros::CallbackQueue *>(root_node_handle_.getCallbackQueue());
            if (queue) {
                queue->callAvailable();
            } else  // in case of nodelets and specific callback queue implementations
            {
                ROS_WARN_ONCE_NAMED(LOGNAME, "Non-default CallbackQueue: Waiting for external queue "
                                             "handling.");
            }
        }
    } else  // wait with timeout
    {
        while (root_node_handle_.ok() && !action->isServerConnected() && timeout > ros::WallTime::now()) {
            ros::WallDuration(0.001).sleep();
            // explicit ros::spinOnce on the callback queue used by NodeHandle that manages the action client
            ros::CallbackQueue *queue = dynamic_cast<ros::CallbackQueue *>(root_node_handle_.getCallbackQueue());
            if (queue) {
                queue->callAvailable();
            } else  // in case of nodelets and specific callback queue implementations
            {
                ROS_WARN_ONCE_NAMED(LOGNAME, "Non-default CallbackQueue: Waiting for external queue "
                                             "handling.");
            }
        }
    }

    if (!action->isServerConnected()) {
        std::stringstream error;
        error << "Unable to connect to action server '" << name << "' within allotted time ("
              << allotted_time
              << "s)";
        throw std::runtime_error(error.str());
        return false;
    } else {
        ROS_DEBUG_NAMED(LOGNAME, "Connected to '%s'", name.c_str());
    }

    return true;
}

}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveDynamicPlanningService, move_group::MoveDynamicCapability)
