//
// Created by spencer on 12/13/22.
//

#ifndef DYNAMIC_FRANKA_PLANNING_SERVICE_CAPABILITY_H
#define DYNAMIC_FRANKA_PLANNING_SERVICE_CAPABILITY_H

#pragma once

#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "move_dynamic_msgs/PlanningAction.h"
#include "move_dynamic_msgs/StartExeAction.h"

#include "moveit/move_group_interface/move_group_interface.h"
#include "move_dynamic/move_group/move_dynamic_capability.h"

namespace move_group
{
class MoveDynamicPlanningService : public MoveDynamicCapability
{
public:
    MoveDynamicPlanningService();

    void initialize() override;

private:
    void executePlanningCallback(const move_dynamic_msgs::PlanningGoalConstPtr& goal);
    void executeGlobalPlanning(const move_dynamic_msgs::PlanningGoalConstPtr& goal,
                               move_dynamic_msgs::PlanningResult& action_res);
    void executeLocalPlanning(const move_dynamic_msgs::PlanningGoalConstPtr& goal,
                              move_dynamic_msgs::PlanningResult& action_res);
    bool connectServer();

    template <typename T>
    bool waitForAction(const T &action, const std::string &name, const ros::WallTime &timeout,
                                                            double allotted_time) const;
    void updatePlan(const int index);
    std::vector<int> determineStartStates();
    void breakTrajectory(bool global = true);
    void sendStartExeWptsSignal();

    bool is_connected_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit_msgs::RobotTrajectory global_trajectory_;
    moveit_msgs::RobotTrajectory local_trajectory_;
    std::vector<moveit_msgs::RobotTrajectory> local_trajectories_;
    std::unique_ptr<actionlib::SimpleActionServer<move_dynamic_msgs::PlanningAction> > planning_action_server_;
    std::unique_ptr<actionlib::SimpleActionClient<move_dynamic_msgs::StartExeAction> > exe_wpts_action_client_;
};
}

#endif //DYNAMIC_FRANKA_PLANNING_SERVICE_CAPABILITY_H
