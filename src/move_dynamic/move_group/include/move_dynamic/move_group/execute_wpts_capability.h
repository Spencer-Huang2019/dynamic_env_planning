//
// Created by spencer on 12/13/22.
//

#ifndef DYNAMIC_FRANKA_EXECUTE_WPTS_CAPABILITY_H
#define DYNAMIC_FRANKA_EXECUTE_WPTS_CAPABILITY_H

#pragma once

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "move_dynamic_msgs/StartExeAction.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "move_dynamic/move_group/move_dynamic_capability.h"

namespace move_group
{
class MoveDynamicExecuteWptsAction : public MoveDynamicCapability
{
public:
    MoveDynamicExecuteWptsAction();

    void initialize() override;

private:
    void executeWptsCallback(const move_dynamic_msgs::StartExeGoalConstPtr& goal);
    void publishTrajectory();
    bool connectServer();

    template <typename T>
    bool waitForAction(const T &action, const std::string &name, const ros::WallTime &timeout,
                       double allotted_time) const;

    void execute(const moveit_msgs::RobotTrajectory& robotTrajectory);

    bool is_connected_;
    std::unique_ptr<actionlib::SimpleActionServer<move_dynamic_msgs::StartExeAction>> exe_wpts_action_server_;
    std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> execute_action_client_;

};
}

#endif //DYNAMIC_FRANKA_EXECUTE_WPTS_CAPABILITY_H
