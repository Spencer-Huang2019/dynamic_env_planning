#!/usr/bin/env python
import time

import rospy
import sys
import move_dynamic_commander
import numpy as np
import matplotlib.pyplot as plt
import yaml
import argparse
from scipy.interpolate import CubicSpline
import geometry_msgs.msg, shape_msgs.msg
import moveit_commander


def dict2namespace(config):
    namespace = argparse.Namespace()
    for key, value in config.items():
        if isinstance(value, dict):
            new_value = dict2namespace(value)
        else:
            new_value = value

        setattr(namespace, key, new_value)
    return vars(namespace)


def load_yaml(filename, filepath):
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg', type=str, default=filename, help='...')
    args = parser.parse_args()

    with open(filepath, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    combine = dict(list(config.items()) + list(vars(args).items()))
    configs = dict2namespace(combine)

    return configs


def load_breakup_traj():
    filepath = "src/franka_move/files/combine_traj/"
    yamlname = "combine_traj.yaml"
    yamlpath = filepath + yamlname
    # txtpoints = filepath + "combine_traj.txt"
    # txtts = filepath + "combine_ts.txt"
    configs = load_yaml(yamlname, yamlpath)
    positions = []
    velocities = []
    accelerations = []
    ts = [0]

    trajs_nums = sorted(configs.keys())

    for traj_num in trajs_nums:
        traj = configs[traj_num]
        if isinstance(traj, str):
            continue
        point0 = traj['point_0']
        point1 = traj['point_1']
        positions.append(point0['positions'])
        velocities.append(point0['velocities'])
        accelerations.append(point0['accelerations'])
        ts.append(ts[-1] + point1['time_from_start'])

    ts.pop(-1)
    # np.savetxt(txtpoints, np.array(points))
    # np.savetxt(txtts, np.array(ts))

    return np.array(positions), np.array(velocities), np.array(accelerations), np.array(ts)


def load_traj():
    filepath = "src/franka_move/files/global_traj/"
    yamlname = "global_traj.yaml"
    yamlpath = filepath + yamlname
    # txtpoints = filepath + "global_traj.txt"
    # txtts = filepath + "global_ts.txt"
    configs = load_yaml(yamlname, yamlpath)
    positions = []
    velocities = []
    accelerations = []
    ts = []

    points_nums = sorted(configs.keys())

    for point_num in points_nums:
        point = configs[point_num]
        if isinstance(point, str):
            continue
        positions.append(point['positions'])
        velocities.append(point['velocities'])
        accelerations.append(point['accelerations'])
        ts.append(point['time_from_start'])

    # np.savetxt(txtpoints, np.array(points))
    # np.savetxt(txtts, np.array(ts))

    return np.array(positions), np.array(velocities), np.array(accelerations), np.array(ts)


def draw_trajs():
    c_qs_sample, c_qds_sample, c_qdds_sample, c_ts_sample = load_breakup_traj()
    g_qs_sample, g_qds_sample, g_qdds_sample, g_ts_sample = load_traj()

    # c_qs_func = CubicSpline(c_ts_sample, c_qs_sample)
    # c_qds_func = c_qs_func.derivative()
    # c_qds_sample = c_qds_func(c_ts_sample)
    #
    # g_qs_func = CubicSpline(g_ts_sample, g_qs_sample)
    # g_qds_func = g_qs_func.derivative()
    # g_qds_sample = g_qds_func(g_ts_sample)
    #
    # c_qds_func = CubicSpline(c_ts_sample, c_qds_sample)
    # c_qdds_func = c_qds_func.derivative()
    # c_qdds_sample = c_qdds_func(c_ts_sample)
    #
    # g_qds_func = CubicSpline(g_ts_sample, g_qds_sample)
    # g_qdds_func = g_qds_func.derivative()
    # g_qdds_sample = g_qdds_func(g_ts_sample)

    c_qdds_func = CubicSpline(c_ts_sample, c_qdds_sample)
    c_qddds_func = c_qdds_func.derivative()
    c_qddds_sample = c_qddds_func(c_ts_sample)

    g_qdds_func = CubicSpline(g_ts_sample, g_qdds_sample)
    g_qddds_func = g_qdds_func.derivative()
    g_qddds_sample = g_qddds_func(g_ts_sample)

    fig1, axs1 = plt.subplots(4, 1, sharex=True)
    for i in range(7):
        # plot the i-th joint trajectory
        axs1[0].plot(g_ts_sample, g_qs_sample[:, i], c="C{:d}".format(i))
        axs1[1].plot(g_ts_sample, g_qds_sample[:, i], c="C{:d}".format(i))
        axs1[2].plot(g_ts_sample, g_qdds_sample[:, i], c="C{:d}".format(i))
        axs1[3].plot(g_ts_sample, g_qddds_sample[:, i], c="C{:d}".format(i))

    axs1[0].set_ylabel("Positions (rad)")
    axs1[1].set_ylabel("Velocities (rad/s)")
    axs1[2].set_ylabel("Accelerations (rad/s2)")
    axs1[3].set_ylabel("Jerks (rad/s3)")
    axs1[3].set_xlabel("Time(s)")
    fig1.suptitle("Global Trajectory")

    fig2, axs2 = plt.subplots(4, 1, sharex=True)
    for i in range(7):
        # plot the i-th joint trajectory
        axs2[0].plot(c_ts_sample, c_qs_sample[:, i], c="C{:d}".format(i))
        axs2[1].plot(c_ts_sample, c_qds_sample[:, i], c="C{:d}".format(i))
        axs2[2].plot(c_ts_sample, c_qdds_sample[:, i], c="C{:d}".format(i))
        axs2[3].plot(c_ts_sample, c_qddds_sample[:, i], c="C{:d}".format(i))

    axs2[0].set_ylabel("Positions (rad)")
    axs2[1].set_ylabel("Velocities (rad/s)")
    axs2[2].set_ylabel("Accelerations (rad/s2)")
    axs2[3].set_ylabel("Jerks (rad/s3)")
    axs2[3].set_xlabel("Time(s)")
    fig2.suptitle("Combine Trajectory")

    plt.show()


class MoveDynamicPythonInterface(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveDynamicPythonInterface, self).__init__()

        move_dynamic_commander.roscpp_initialize(sys.argv)
        rospy.init_node("franka_move_start", anonymous=True)

        move_dynamic = move_dynamic_commander.MoveDynamicCommander()
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        scene.remove_world_object()

        self.resample_dt = 0.09
        self.scaling_factor = 0.5
        self.move_dynamic = move_dynamic
        self.scene = scene

    def run_planning(self, replanning=True, joint_goal=None):
        # global planning
        params = self.get_params(replanning, joint_goal=joint_goal, resample_dt=self.resample_dt, scaling_factor=self.scaling_factor)
        result = self.move_dynamic.plan(params)

        return result

    def add_obstacles(self, obstacles):
        self.move_dynamic.clear_obstacles()
        for (box_pose, box_size) in obstacles:
            self.add_one_box(box_pose, box_size)

    def get_params(self, replanning, joint_goal=None, resample_dt=None, scaling_factor=None):
        params = {}
        if replanning:
            params['replanning'] = True
            params['planner_id'] = "ERRTConnect"
        else:
            params['replanning'] = False
            params['planner_id'] = "RRTConnect"
            params['joint_position'] = joint_goal

        params['resample_dt'] = resample_dt
        params['scaling_factor'] = scaling_factor

        return params

    def add_one_box(self, box_pose, box_size):
        p = geometry_msgs.msg.PoseStamped()
        p.pose.position.x = box_pose[0]
        p.pose.position.y = box_pose[1]
        p.pose.position.z = box_pose[2]
        p.pose.orientation.x = box_pose[3]
        p.pose.orientation.y = box_pose[4]
        p.pose.orientation.z = box_pose[5]
        p.pose.orientation.w = box_pose[6]

        primitive = shape_msgs.msg.SolidPrimitive()
        primitive.type = primitive.BOX
        primitive.dimensions = [0.5] * 3
        primitive.dimensions[primitive.BOX_X] = box_size[0]
        primitive.dimensions[primitive.BOX_Y] = box_size[1]
        primitive.dimensions[primitive.BOX_Z] = box_size[2]

        self.move_dynamic.add_obstacle(p, primitive)


def generate_new_problem(seed=9):
    # Parameters
    way_pts = [
        [-0.185079, 0.41187, -0.421092, -2.4302, -0.14004, 3.27095, 0.878958],
        [-0.224502, 0.666343, -0.510116, -2.4468, -0.196572, 3.63293, 0.898124]]
    s1 = 0
    s2 = 0.0295465
    ss = [s1, s2]
    return (
        ss,
        way_pts,
        [[-1.0875, 1.0875], [-1.0875, 1.0875], [-1.0875, 1.0875], [-1.0875, 1.0875], [-1.305, 1.305], [-1.305, 1.305], [-1.305, 1.305]],
        [[-1.875, 1.875], [-0.9375, 0.9375], [-1.25, 1.25], [-1.5625, 1.5625], [-1.875, 1.875], [-2.5, 2.5], [-2.5, 2.5]],
    )


if __name__ == '__main__':

    dynamic_planning = MoveDynamicPythonInterface()

    joint_goal1 = [-0.2244, 0.6672, -0.5111, -2.4468, -0.1701, 3.6322, 0.8989]
    joint_goal2 = [-0.00015, -0.78555, 7.7980e-05, -2.35599, 4.47189e-05, 1.571559, 0.785379]

    """ adding obstacles """
    box_pose = [0.5, 0.0, 0.25, 0, 0, 0, 1]
    box_size = [0.1, 1.5, 0.5]
    obstacles = [(box_pose, box_size)]
    dynamic_planning.add_obstacles(obstacles)

    """ run global planning """
    plan_glo_ret = dynamic_planning.run_planning(replanning=False, joint_goal=joint_goal1)

    # """ adding obstacles """
    # box_pose = [0.3, 0.0, 0.9, 0, 0, 0, 1]
    # box_size = [0.1, 0.1, 0.1]
    # obstacles = [(box_pose, box_size)]
    # dynamic_planning.add_obstacles(obstacles)

    # """ run local planning """
    # if plan_glo_ret:
    #     plan_loc_ret = dynamic_planning.run_planning(replanning=True)
    #
    # """ run global planning """
    # dynamic_planning.run_planning(replanning=False, joint_goal=joint_goal2)

# draw_trajs()
