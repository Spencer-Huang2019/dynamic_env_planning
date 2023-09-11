//
// Created by spencer on 12/26/22.
//

#include "plan_multi_action_capability.h"

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/utils/message_checks.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/plan_execution/plan_execution.h>

namespace move_group
{
MoveGroupPlanMultiAction::MoveGroupPlanMultiAction()
: MoveGroupCapability("PlanMulti"), move_state_(IDLE), preempt_requested_{false}
{
}

void MoveGroupPlanMultiAction::initialize()
{
    plan_multi_action_server_ = std::make_unique<actionlib::SimpleActionServer<moveit_msgs::MoveDynamicAction>>(
            root_node_handle_, PLAN_ACTION, [this](const auto& goal) { executePlanMultiCallback(goal);}, false);
    plan_multi_action_server_->registerPreemptCallback([this] {preemptPlanMultiCallback();});
    plan_multi_action_server_->start();
}

void MoveGroupPlanMultiAction::executePlanMultiCallback(const moveit_msgs::MoveDynamicGoalConstPtr& goal)
{
    setMoveState(PLANNING);

    std::size_t num_start_states = goal->request.start_states.size();

    context_->planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
    context_->planning_scene_monitor_->updateFrameTransforms();

    moveit_msgs::MoveDynamicResult action_res;
    executePlanMultiCallbackPlanOnly(goal, action_res);

    bool planned_trajectory_empty = false;
    for (std::size_t i = 0; i < num_start_states; ++i)
    {
        planned_trajectory_empty = trajectory_processing::isTrajectoryEmpty(action_res.planned_trajectories[i]);
        if (planned_trajectory_empty)
            break;
    }

    std::string response =
            getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        plan_multi_action_server_->setSucceeded(action_res, response);
    else
    {
        if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
            plan_multi_action_server_->setPreempted(action_res, response);
        else
            plan_multi_action_server_->setAborted(action_res, response);
    }

    setMoveState(IDLE);

    preempt_requested_ = false;
}

void MoveGroupPlanMultiAction::executePlanMultiCallbackPlanOnly(const moveit_msgs::MoveDynamicGoalConstPtr& goal,
                                                                moveit_msgs::MoveDynamicResult& action_res)
{
    planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
    const planning_scene::PlanningSceneConstPtr& the_scene =
            (moveit::core::isEmpty(goal->planning_options.planning_scene_diff)) ?
            static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) :
            lscene->diff(goal->planning_options.planning_scene_diff);
    planning_interface::MotionPlanMultiResponse res;

    if(preempt_requested_)
    {
        ROS_INFO_NAMED(getName(), "Preempt requested before the goal is planned.");
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
        return;
    }

    const planning_pipeline::PlanningPipelinePtr planning_pipeline = resolvePlanningPipeline(goal->request.pipeline_id);
    if (!planning_pipeline)
    {
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return;
    }

    try
    {
        planning_pipeline->generateMultiPlan(the_scene, goal->request, res);
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_NAMED(getName(), "Planning pipeline threw an exception: %s", ex.what());
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }

    convertToMsg(res.trajectories_, action_res.trajectory_starts, action_res.planned_trajectories);
    action_res.error_code = res.error_code_;
    action_res.planning_time = res.planning_time_;
    action_res.start_index = res.start_index_;
}

void MoveGroupPlanMultiAction::preemptPlanMultiCallback()
{
    preempt_requested_ = true;
    context_->plan_execution_->stop();
}

void MoveGroupPlanMultiAction::setMoveState(MoveGroupState state)
{
    move_state_ = state;
    move_feedback_.state = stateToStr(state);
    plan_multi_action_server_->publishFeedback(move_feedback_);
}
}

#include "class_loader/class_loader.hpp"
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupPlanMultiAction, move_group::MoveGroupCapability)
