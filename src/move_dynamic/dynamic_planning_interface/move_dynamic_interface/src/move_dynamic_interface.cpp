//
// Created by spencer on 1/3/23.
//

#include <moveit/robot_state/conversions.h>
#include <move_dynamic/move_dynamic_interface/move_dynamic_interface.h>

namespace move_dynamic
{
namespace dynamic_planning_interface
{
const std::string LOGNAME = "move_dynamic_interface";

class MoveDynamicInterface::MoveDynamicInterfaceImpl
{
    friend MoveDynamicInterface;

public:
    MoveDynamicInterfaceImpl(const ros::WallDuration& wait_for_servers)
    {
        std::string NAME = "planning_service";
        ros::WallTime timeout_for_servers = ros::WallTime::now() + wait_for_servers;
        if (wait_for_servers == ros::WallDuration())
            timeout_for_servers = ros::WallTime();
        double allotted_time = wait_for_servers.toSec();

        planning_client_ = std::make_unique<actionlib::SimpleActionClient<move_dynamic_msgs::PlanningAction>>(
                node_handle_, NAME, false);
        waitForAction(planning_client_, NAME, timeout_for_servers, allotted_time);
    }

    template <typename T>
    void waitForAction(const T& action, const std::string& name, const ros::WallTime& timeout, double allotted_time) const
    {
        ROS_DEBUG_NAMED(LOGNAME, "Waiting for move_dynamic_planning_interface planning action server (%s)...", name.c_str());

        // wait for the server (and spin as needed)
        if (timeout == ros::WallTime())  // wait forever
        {
            while (node_handle_.ok() && !action->isServerConnected())
            {
                ros::WallDuration(0.001).sleep();
                // explicit ros::spinOnce on the callback queue used by NodeHandle that manages the action client
                ros::CallbackQueue* queue = dynamic_cast<ros::CallbackQueue*>(node_handle_.getCallbackQueue());
                if (queue)
                {
                    queue->callAvailable();
                }
                else  // in case of nodelets and specific callback queue implementations
                {
                    ROS_WARN_ONCE_NAMED(LOGNAME, "Non-default CallbackQueue: Waiting for external queue "
                                                 "handling.");
                }
            }
        }
        else  // wait with timeout
        {
            while (node_handle_.ok() && !action->isServerConnected() && timeout > ros::WallTime::now())
            {
                ros::WallDuration(0.001).sleep();
                // explicit ros::spinOnce on the callback queue used by NodeHandle that manages the action client
                ros::CallbackQueue* queue = dynamic_cast<ros::CallbackQueue*>(node_handle_.getCallbackQueue());
                if (queue)
                {
                    queue->callAvailable();
                }
                else  // in case of nodelets and specific callback queue implementations
                {
                    ROS_WARN_ONCE_NAMED(LOGNAME, "Non-default CallbackQueue: Waiting for external queue "
                                                 "handling.");
                }
            }
        }

        if (!action->isServerConnected())
        {
            std::stringstream error;
            error << "Unable to connect to move_group action server '" << name << "' within allotted time (" << allotted_time
                  << "s)";
            throw std::runtime_error(error.str());
        }
        else
        {
            ROS_DEBUG_NAMED(LOGNAME, "Connected to '%s'", name.c_str());
        }
    }

    ~MoveDynamicInterfaceImpl()
    {
    }

    void setPlannerId(const std::string& planner_id)
    {
        planner_id_ = planner_id;
    }

    void setResampleDt(double resample_dt)
    {
        resample_dt_ = resample_dt;
    }

    void setJointPositionTarget(const std::vector<double> joint_position)
    {
        joint_position_target_.resize(joint_position.size());
        joint_position_target_ = joint_position;
    }

    void setReplanning(bool replanning)
    {
        replanning_ = replanning;
    }

    void setMaxVel(const std::vector<double>& max_vel)
    {
      max_vel_ = max_vel;
    }

    void setMaxAcc(const std::vector<double>& max_acc)
    {
      max_acc_ = max_acc;
    }

    void setMaxJerk(const std::vector<double>& max_jerk)
    {
      max_jerk_ = max_jerk;
    }

    void setScalingFactor(double scaling_factor)
    {
        scaling_factor_ = scaling_factor;
    }

    bool plan()
    {
        if (!planning_client_)
        {
            ROS_ERROR_STREAM_NAMED(LOGNAME, "planning action client not found");
            return false;
        }

        if (!planning_client_->isServerConnected())
        {
            ROS_WARN_STREAM_NAMED(LOGNAME, "planning action server not connected");
            return false;
        }

        move_dynamic_msgs::PlanningGoal goal;
        goal.resample_dt = resample_dt_;
        goal.scaling_factor = scaling_factor_;
        goal.planner_id = planner_id_;
        goal.replanning = replanning_;
        goal.joint_goal = joint_position_target_;
        goal.obstacles = obstacles_;
        goal.max_vel = max_vel_;
        goal.max_acc = max_acc_;
        goal.max_jerk = max_jerk_;

        planning_client_->sendGoal(goal);
        if (!planning_client_->waitForResult())
        {
            ROS_INFO_STREAM_NAMED(LOGNAME, "MoveDynamic planning action returned early");
        }
        if (planning_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            return planning_client_->getResult()->error_code;
        }
        else
        {
            ROS_WARN_STREAM_NAMED(LOGNAME, "Fail: " << planning_client_->getState().toString() << ": "
                                                    << planning_client_->getState().getText());
            return planning_client_->getResult()->error_code;
        }
    }

    void addObstacle(geometry_msgs::Pose& pose, shape_msgs::SolidPrimitive& primitive)
    {
      move_dynamic_msgs::Obstacle obstacle;
      obstacle.pose = pose;
      obstacle.primitive = primitive;
      obstacles_.push_back(obstacle);
    }

    void clearObstacles()
    {
      obstacles_.clear();
    }

private:
    ros::NodeHandle node_handle_;
    std::unique_ptr<actionlib::SimpleActionClient<move_dynamic_msgs::PlanningAction>> planning_client_;
    moveit::core::RobotStatePtr considered_start_state_;
    std::string planner_id_{"ERRTConnect"};
    double resample_dt_{0.01};
    double scaling_factor_{0.5};
    std::vector<double> max_vel_;
    std::vector<double> max_acc_;
    std::vector<double> max_jerk_;
    std::vector<double> joint_position_target_;
    bool replanning_{true};
    std::vector<move_dynamic_msgs::Obstacle> obstacles_;
};

MoveDynamicInterface::MoveDynamicInterface(const ros::WallDuration& wait_for_servers)
{
    if (!ros::ok())
        throw std::runtime_error("ROS does not seem to be running");
    impl_ = new MoveDynamicInterfaceImpl(ros::WallDuration(wait_for_servers.toSec()));
}

MoveDynamicInterface::~MoveDynamicInterface()
{
    delete impl_;
}

MoveDynamicInterface::MoveDynamicInterface(MoveDynamicInterface&& other) noexcept
: impl_(other.impl_)
{
other.impl_ = nullptr;
}

MoveDynamicInterface& MoveDynamicInterface::operator=(MoveDynamicInterface&& other) noexcept
{
    if (this != &other)
    {
        delete impl_;
        impl_ = other.impl_;
        other.impl_ = nullptr;
    }

    return *this;
}

bool MoveDynamicInterface::plan()
{
    impl_->plan();
}

void MoveDynamicInterface::setPlannerId(const std::string& planner_id)
{
    impl_->setPlannerId(planner_id);
}

void MoveDynamicInterface::setScalingFactor(double scaling_factor)
{
    impl_->setScalingFactor(scaling_factor);
}

void MoveDynamicInterface::setResampleDt(double resample_dt)
{
    impl_->setResampleDt(resample_dt);
}

void MoveDynamicInterface::setMaxVel(const std::vector<double>& max_vel)
{
  impl_->setMaxVel(max_vel);
}

void MoveDynamicInterface::setMaxAcc(const std::vector<double>& max_acc)
{
  impl_->setMaxAcc(max_acc);
}

void MoveDynamicInterface::setMaxJerk(const std::vector<double>& max_jerk)
{
  impl_->setMaxJerk(max_jerk);
}

void MoveDynamicInterface::setJointPositionTarget(const std::vector<double> joint_position)
{
    impl_->setJointPositionTarget(joint_position);
}

void MoveDynamicInterface::setReplanning(bool replanning)
{
    impl_->setReplanning(replanning);
}

void MoveDynamicInterface::addObstacle(geometry_msgs::Pose& pose, shape_msgs::SolidPrimitive& primitive)
{
  impl_->addObstacle(pose, primitive);
}

void MoveDynamicInterface::clearObstacles()
{
  impl_->clearObstacles();
}
}
}