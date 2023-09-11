"""Retime a path subject to kinematic constraints
=================================================

In this example, we will see how can we retime a generic spline-based
path subject to kinematic constraints. This is very simple to do with
`toppra`, as we shall see below. First import the library.

"""

import sys
sys.path.append("/home/spencer/workspaces/dynamic_env_planning_ws/src/topp/stopp")

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import math
import numpy as np

ta.setup_logging("INFO")


def get_sample_times(sample_count, duration, resample_dt):
    ts = []
    for i in range(sample_count + 1):
        t = min(duration, i * resample_dt)
        ts.append(t)

    return ts


def call_stopp(way_pts, ss, vlims, alims, jlims, resample_dt, init_path_vel, init_qs):

    #print('way_pts', way_pts)
    #print('ss', ss)
    print('vlims', vlims)
    print('alims', alims)
    print('jlims', jlims)
    #print('init_path_vel', init_path_vel)

    # Define the geometric path and two constraints.
    if init_path_vel:
        # print('init_qs', init_qs)
        path = ta.SplineInterpolator(ss, way_pts, init_qs)
    else:
        path = ta.SplineInterpolator(ss, way_pts)
    pc_vel = constraint.JointVelocityConstraint(vlims)
    pc_acc = constraint.JointAccelerationConstraint(alims)
    pc_jerk = constraint.JointJerkConstraint(jlims)

    ################################################################################
    # We solve the parametrization problem using the
    # `ParametrizeConstAccel` parametrizer. This parametrizer is the
    # classical solution, guarantee constraint and boundary conditions
    # satisfaction.
    instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
    _, sd_ra, _ = instance.compute_parameterization(init_path_vel, 0)
    instance_j = algo.SamplingAlgorithmV3([pc_acc.formal, pc_jerk.formal], path, n_samples=5,
                                    xs_max=(sd_ra ** 2).reshape(-1, 1), parametrizer="ParametrizeConstAccel")
    jnt_traj = instance_j.compute_trajectory_jerk(init_path_vel, 0)
    if jnt_traj:
        duration = jnt_traj.duration
        # print('duration', duration)
        sample_count = math.ceil(duration / resample_dt)
        ts_sample = get_sample_times(int(sample_count), duration, resample_dt)

        qs_sample = jnt_traj(ts_sample)
        qds_sample = jnt_traj(ts_sample, 1)
        qdds_sample = jnt_traj(ts_sample, 2)
        path_velocities = jnt_traj.path_velocities

        # print('ts_sample', ts_sample)
        # print('path_velocities', path_velocities.tolist())
        # print('qs_sample', qs_sample.tolist())

        return ts_sample, qs_sample.tolist(), qds_sample.tolist(), qdds_sample.tolist(), path_velocities.tolist()
    else:
        return None, None, None, None, None
