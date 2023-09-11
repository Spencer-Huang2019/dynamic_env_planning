//
// Created by spencer on 12/26/22.
//

#ifndef DYNAMIC_FRANKA_PLAN_MULTI_ACTION_CAPABILITY_H
#define DYNAMIC_FRANKA_PLAN_MULTI_ACTION_CAPABILITY_H

#pragma once

#include "moveit/move_group/move_group_capability.h"
#include "actionlib/server/simple_action_server.h"
#include "moveit_msgs/MoveDynamicAction.h"
#include "memory"

namespace move_group
{
class MoveGroupPlanMultiAction : public MoveGroupCapability
{
public:
    MoveGroupPlanMultiAction();

    void initialize() override;

private:
    void executePlanMultiCallback(const moveit_msgs::MoveDynamicGoalConstPtr& goal);
    void executePlanMultiCallbackPlanOnly(const moveit_msgs::MoveDynamicGoalConstPtr& goal,
                                          moveit_msgs::MoveDynamicResult& action_res);
    void preemptPlanMultiCallback();
    void setMoveState(MoveGroupState state);

    std::unique_ptr<actionlib::SimpleActionServer<moveit_msgs::MoveDynamicAction>> plan_multi_action_server_;
    moveit_msgs::MoveDynamicFeedback move_feedback_;

    MoveGroupState move_state_;
    bool preempt_requested_;
};
}

#endif //DYNAMIC_FRANKA_PLAN_MULTI_ACTION_CAPABILITY_H
