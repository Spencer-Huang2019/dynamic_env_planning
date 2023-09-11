//
// Created by spencer on 12/13/22.
//

#ifndef DYNAMIC_FRANKA_MOVE_DYNAMIC_CAPABILITY_H
#define DYNAMIC_FRANKA_MOVE_DYNAMIC_CAPABILITY_H

#pragma once

#include "moveit/planning_interface/planning_interface.h"
#include <moveit/macros/class_forward.h>

namespace move_group
{

MOVEIT_CLASS_FORWARD(MoveDynamicCapability);

class MoveDynamicCapability
{
public:
    MoveDynamicCapability(const std::string& capability_name) : node_handle_("~"), capability_name_(capability_name)
    {
        if (!node_handle_.getParam("/group_name", group_name_))
        {
            ROS_ERROR_STREAM("No group name found!");
        }
    }

    virtual ~MoveDynamicCapability(){}

    virtual void initialize() = 0;

    const std::string& getName() const
    {
        return capability_name_;
    }

protected:

    ros::NodeHandle root_node_handle_;
    ros::NodeHandle node_handle_;
    std::string capability_name_;
    std::string group_name_;
    static std::vector<moveit_msgs::RobotTrajectory> trajectories_;
};
}


#endif //DYNAMIC_FRANKA_MOVE_DYNAMIC_CAPABILITY_H
