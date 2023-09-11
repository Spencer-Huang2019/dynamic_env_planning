from move_dynamic_planning_interface import _move_dynamic_interface
import moveit_commander.conversions as conversions

class MoveDynamicCommander(object):

    def __init__(self, wait_for_servers=5.0):
        self._g = _move_dynamic_interface.MoveDynamicInterface(wait_for_servers)

    def set_planner_id(self, planner_id):
        self._g.set_planner_id(planner_id)

    def set_resample_dt(self, resample_dt):
        self._g.set_resample_dt(resample_dt)

    def set_joint_position_target(self, joint_position):
        self._g.set_joint_position_target(joint_position)

    def set_replanning(self, replanning):
        self._g.set_replanning(replanning)

    def set_scaling_factor(self, scaling_factor):
        self._g.set_scaling_factor(scaling_factor)

    def set_joint_max_vel(self, values):
        self._g.set_joint_max_vel(values)

    def set_joint_max_acc(self, values):
        self._g.set_joint_max_acc(values)

    def set_joint_max_jerk(self, values):
        self._g.set_joint_max_jerk(values)

    def plan(self, params=None):
        if params.has_key('resample_dt'):
            self.set_resample_dt(params['resample_dt'])
        if params.has_key('scaling_factor'):
            self.set_scaling_factor(params['scaling_factor'])
        if params.has_key('planner_id'):
            self.set_planner_id(params['planner_id'])
        if params.has_key('joint_position'):
            self.set_joint_position_target(params['joint_position'])
        if params.has_key('replanning'):
            self.set_replanning(params['replanning'])
        if params.has_key('max_vel'):
            self.set_joint_max_vel(params['max_vel'])
        if params.has_key('max_acc'):
            self.set_joint_max_acc(params['max_acc'])
        if params.has_key('max_jerk'):
            self.set_joint_max_jerk(params['max_jerk'])

        result = self._g.plan()

        return result

    def add_obstacle(self, pose, primitive):
        self._g.add_obstacle(conversions.msg_to_string(pose), conversions.msg_to_string(primitive))

    def clear_obstacles(self):
        self._g.clear_obstacles()