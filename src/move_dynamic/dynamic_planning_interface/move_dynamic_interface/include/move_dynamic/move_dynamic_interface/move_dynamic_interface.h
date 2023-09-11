//
// Created by spencer on 1/3/23.
//

#ifndef DYNAMIC_FRANKA_MOVE_DYNAMIC_INTERFACE_H
#define DYNAMIC_FRANKA_MOVE_DYNAMIC_INTERFACE_H

#pragma once


//#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/macros/class_forward.h>
#include "move_dynamic_msgs/PlanningAction.h"
#include "actionlib/client/simple_action_client.h"

namespace move_dynamic
{
namespace dynamic_planning_interface
{
MOVEIT_CLASS_FORWARD(MoveDynamicInterface);

class MoveDynamicInterface
{
public:
    static const std::string ROBOT_DESCRIPTION;

    MoveDynamicInterface(const ros::WallDuration& wait_for_servers = ros::WallDuration());

    ~MoveDynamicInterface();

    MoveDynamicInterface(const MoveDynamicInterface&) = delete;
    MoveDynamicInterface& operator=(const MoveDynamicInterface&) = delete;

    MoveDynamicInterface(MoveDynamicInterface&& other) noexcept;
    MoveDynamicInterface& operator=(MoveDynamicInterface&& other) noexcept;

    bool plan();
    void setPlannerId(const std::string& planner_id);
    void setResampleDt(double resample_dt);
    void setJointPositionTarget(const std::vector<double> joint_position);
    void setReplanning(bool replanning);
    void setScalingFactor(double scaling_factor);
    void addObstacle(geometry_msgs::Pose& pose, shape_msgs::SolidPrimitive& primitive);
    void clearObstacles();
    void setMaxVel(const std::vector<double>& values);
    void setMaxAcc(const std::vector<double>& values);
    void setMaxJerk(const std::vector<double>& values);

    ros::NodeHandle node_handle_;

private:
    class MoveDynamicInterfaceImpl;
    MoveDynamicInterfaceImpl* impl_;
};
}
}

#endif //DYNAMIC_FRANKA_MOVE_DYNAMIC_INTERFACE_H
