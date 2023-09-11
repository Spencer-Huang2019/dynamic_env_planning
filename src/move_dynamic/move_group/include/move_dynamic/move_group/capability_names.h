//
// Created by spencer on 12/13/22.
//

#ifndef DYNAMIC_FRANKA_CAPABILITY_NAMES_H
#define DYNAMIC_FRANKA_CAPABILITY_NAMES_H

#pragma once

#include "string"

namespace move_group
{
static const std::string PLANNING_SERVICE_NAME = "planning_service";
static const std::string EXECUTE_WPTS_NAME = "execute_waypoints";
static const std::string FOLLOW_JOINT_TRAJ_NAME = "/effort_joint_trajectory_controller/follow_joint_trajectory";
}

#endif //DYNAMIC_FRANKA_CAPABILITY_NAMES_H
