//
// Created by spencer on 12/13/22.
//

#include "move_dynamic/move_group/execute_wpts_capability.h"
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include "move_dynamic/move_group/capability_names.h"

constexpr char LOGNAME[] = "Execution_Capability";

namespace move_group
{
MoveDynamicExecuteWptsAction::MoveDynamicExecuteWptsAction()
: MoveDynamicCapability("ExecuteWptsAction")
{
}

void MoveDynamicExecuteWptsAction::initialize()
{
    exe_wpts_action_server_ = std::make_unique<actionlib::SimpleActionServer<move_dynamic_msgs::StartExeAction>>(
            root_node_handle_, EXECUTE_WPTS_NAME, [this](const auto& goal){executeWptsCallback(goal);}, false);
    exe_wpts_action_server_->start();
}

void MoveDynamicExecuteWptsAction::executeWptsCallback(const move_dynamic_msgs::StartExeGoalConstPtr& goal)
{
    exe_wpts_action_server_->setSucceeded();
    is_connected_ = connectServer();
    publishTrajectory();
}

void MoveDynamicExecuteWptsAction::publishTrajectory()
{
    while (!is_connected_)
    {
        is_connected_ = connectServer();
    }

//    ROS_INFO_STREAM("trajectories_ size in execution: " << trajectories_.size());
    int i = 0;
    while (trajectories_.size() > i && ros::ok)
    {
        execute(trajectories_[i]);
        root_node_handle_.setParam("move_dynamic_interface/execute/execution_index", i);
        ros::Duration(trajectories_[i].joint_trajectory.points[1].time_from_start).sleep();
        i++;
    }
    ROS_INFO_STREAM("Finish publishing waypoints. ");
}

void MoveDynamicExecuteWptsAction::execute(const moveit_msgs::RobotTrajectory &robotTrajectory) {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = robotTrajectory.joint_trajectory;

    execute_action_client_->sendGoal(goal);
}

bool MoveDynamicExecuteWptsAction::connectServer()
{
    std::string name = FOLLOW_JOINT_TRAJ_NAME;

    ros::WallDuration wait_for_servers = ros::WallDuration(2);
    ros::WallTime timeout_for_servers = ros::WallTime::now() + wait_for_servers;
    if (wait_for_servers == ros::WallDuration())
        timeout_for_servers = ros::WallTime();  // wait for ever
    double allotted_time = wait_for_servers.toSec();

    execute_action_client_ =
            std::make_unique < actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction>> (
                    root_node_handle_, name, false);
    if (waitForAction(execute_action_client_, name, timeout_for_servers, allotted_time))
    {
        return true;
    }
    else
    {
        return false;
    }

}

template <typename T>
bool MoveDynamicExecuteWptsAction::waitForAction(const T &action, const std::string &name, const ros::WallTime &timeout,
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
CLASS_LOADER_REGISTER_CLASS(move_group::MoveDynamicExecuteWptsAction, move_group::MoveDynamicCapability)