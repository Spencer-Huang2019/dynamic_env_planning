
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/toppra_generation.h>
#include <class_loader/class_loader.hpp>
#include <ros/console.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

namespace default_planner_request_adapters
{
using namespace trajectory_processing;
const std::string LOGNAME = "TOPPRA";

/** @brief This adapter uses the time-optimal trajectory generation method */
class AddToppra : public planning_request_adapter::PlanningRequestAdapter
{
public:
    AddToppra() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const ros::NodeHandle& nodeHandle) override
  {
    nh_ = nodeHandle;
  }

  std::string getDescription() const override
  {
    return "Add TOPPRA";
  }

  float getSampleDuration() const
  {
    float sampleDuration = 0.1;
    if (!nh_.getParam("resample_dt", sampleDuration))
    {
      return sampleDuration;
    }
    else
    {
      return sampleDuration;
    }
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory_)
    {
        ROS_INFO_NAMED(LOGNAME, "Running '%s'", getDescription().c_str());
        ToppraGeneration topp_ra(getSampleDuration(), 0.001);
        if (!topp_ra.computeTrajectory(*res.trajectory_, req.max_velocity_scaling_factor,
                               req.max_acceleration_scaling_factor))
        {
            ROS_ERROR_NAMED(LOGNAME, "Time parametrization for the solution path failed.");
            result = false;
        }
    }

    return result;
  }

    bool adaptAndPlan(const MultiPlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                      const planning_interface::MotionPlanMultiRequest& req, planning_interface::MotionPlanMultiResponse& res,
                      std::vector<std::size_t>& /*added_path_index*/) const override
    {
        bool result = planner(planning_scene, req, res);
        int start_index = res.start_index_;

        if (result && res.trajectories_[start_index])
        {
            ROS_INFO_NAMED(LOGNAME, "Running '%s'", getDescription().c_str());
            moveit::core::RobotState start_state = res.start_states_[start_index];
            res.trajectories_[start_index]->addPrefixWayPoint(start_state, 0);
            moveit::core::RobotStatePtr start_state_ptr(new moveit::core::RobotState(start_state));
            double *positions = start_state_ptr->getVariablePositions();

            ToppraGeneration topp_ra(getSampleDuration(), 0.001);
            if (!topp_ra.computeTrajectory(*res.trajectories_[start_index], req.max_velocity_scaling_factor,
                                           req.max_acceleration_scaling_factor, start_state_ptr)) {
                ROS_ERROR_NAMED(LOGNAME, "Time parametrization for the solution path failed.");
            }
        }

        return result;
    }
};

}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::AddToppra,
                            planning_request_adapter::PlanningRequestAdapter);
