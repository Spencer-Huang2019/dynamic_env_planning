//
// Created by spencer on 1/3/23.
//

#include <move_dynamic/move_dynamic_interface/move_dynamic_interface.h>
#include <move_dynamic/py_bindings_tools/roscpp_initializer.h>
#include <move_dynamic/py_bindings_tools/py_conversions.h>
#include <move_dynamic/py_bindings_tools/serialize_msg.h>
#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <Python.h>
#include <memory>

namespace bp = boost::python;

//using moveit::py_bindings_tools::GILReleaser;

namespace move_dynamic
{
namespace dynamic_planning_interface
{
class MoveDynamicInterfaceWrapper : protected py_bindings_tools::ROScppInitializer, public MoveDynamicInterface {
public:
    MoveDynamicInterfaceWrapper(double wait_for_servers = 5.0)
            : py_bindings_tools::ROScppInitializer(), MoveDynamicInterface(ros::WallDuration(wait_for_servers)) {
    }

    void setJointMaxVelPython(bp::list& values)
    {
      setMaxVel(py_bindings_tools::doubleFromList(values));
    }

    void setJointMaxAccPython(bp::list& values)
    {
      setMaxAcc(py_bindings_tools::doubleFromList(values));
    }

    void setJointMaxJerkPython(bp::list& values)
    {
      setMaxJerk(py_bindings_tools::doubleFromList(values));
    }

    void setJointPositionTargetPython(bp::list& joint_positions)
    {
        MoveDynamicInterface::setJointPositionTarget(py_bindings_tools::doubleFromList(joint_positions));
    }

    void addObstaclePython(const py_bindings_tools::ByteString& pose_str,
                           const py_bindings_tools::ByteString& primitive_str)
    {
      geometry_msgs::Pose pose_msg;
      shape_msgs::SolidPrimitive primitive_msg;
      py_bindings_tools::deserializeMsg(pose_str, pose_msg);
      py_bindings_tools::deserializeMsg(primitive_str, primitive_msg);
      MoveDynamicInterface::addObstacle(pose_msg, primitive_msg);
    }

};

static void wrap_move_dynamic_interface() {
  eigenpy::enableEigenPy();
  bp::class_<MoveDynamicInterfaceWrapper, boost::noncopyable> move_dynamic_interface_class(
          "MoveDynamicInterface", bp::init < bp::optional < double >> ());

  move_dynamic_interface_class.def("plan", &MoveDynamicInterfaceWrapper::plan);
  move_dynamic_interface_class.def("set_planner_id", &MoveDynamicInterfaceWrapper::setPlannerId);
  move_dynamic_interface_class.def("set_resample_dt", &MoveDynamicInterfaceWrapper::setResampleDt);
  move_dynamic_interface_class.def("set_joint_max_vel", &MoveDynamicInterfaceWrapper::setJointMaxVelPython);
  move_dynamic_interface_class.def("set_joint_max_acc", &MoveDynamicInterfaceWrapper::setJointMaxAccPython);
  move_dynamic_interface_class.def("set_joint_max_jerk", &MoveDynamicInterfaceWrapper::setJointMaxJerkPython);
  move_dynamic_interface_class.def("set_joint_position_target",
                                   &MoveDynamicInterfaceWrapper::setJointPositionTargetPython);
  move_dynamic_interface_class.def("set_replanning", &MoveDynamicInterfaceWrapper::setReplanning);
  move_dynamic_interface_class.def("set_scaling_factor", &MoveDynamicInterfaceWrapper::setScalingFactor);
  move_dynamic_interface_class.def("add_obstacle", &MoveDynamicInterfaceWrapper::addObstaclePython);
  move_dynamic_interface_class.def("clear_obstacles", &MoveDynamicInterfaceWrapper::clearObstacles);
}
}
}

BOOST_PYTHON_MODULE(_move_dynamic_interface)
{
    using namespace move_dynamic::dynamic_planning_interface;
    wrap_move_dynamic_interface();
}