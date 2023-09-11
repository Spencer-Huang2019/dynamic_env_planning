"""Retime a path subject to kinematic constraints
=================================================

In this example, we will see how can we retime a generic spline-based
path subject to kinematic constraints. This is very simple to do with
`toppra`, as we shall see below. First import the library.

"""
# import os
#
# path = '../toppra/solverwrapper/cy_seidel_solverwrapper.c'
# if os.path.exists(path):
#     os.remove(path)
#
# cd = "cd D:/research/projects/TOPP/TOPP_RA/toppra-develop/"
# adb = "python setup.py build_ext --inplace"
# d = os.popen(cd + "&&" + adb)
# f = d.read()
# print(f)


import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.interpolate import CubicSpline

ta.setup_logging("INFO")

################################################################################
# We generate a path with some random waypoints.

def generate_new_problem(seed=9):
    # Parameters
    N_samples = 6
    dof = 6
    np.random.seed(seed)
    # way_pts = [[3.1415927410126, 3.1415967941284, 3.1415929794312, 3.1415920257568, 3.1415927410126, 3.1415922641754],
    #            [2.9806101322174, 3.0475478172302, 3.1409873962402, 3.1076600551605, 3.1985490322113, 3.2243940830231],
    #            [3.2473683357239, 3.2953672409058, 2.792797088623, 2.9378292560577, 3.1403820514679, 3.073723077774],
    #            [2.4440011978149, 2.7340662479401, 3.1392576694489, 3.0106976032257, 3.3612802028656, 3.4609711170197],
    #            [3.572830915451, 3.7685213088989, 1.7195794582367, 2.3108658790588, 3.1369225978851, 2.879798412323],
    #            [3.1329445838928, 2.6567847728729, 3.9552495479584, 4.3244771957397, 0.45854878425598, 1.574184179306]]
    way_pts = [[-1.8227082490921, -0.2303059399128, -1.4795190095901, 0.14013521373272, -1.5703538656235, 0.42700237035751],
               [-1.5997982025146, -0.19523184001446, -1.7860630750656, 0.41355004906654, -1.5717170238495, 0.29123303294182],
               [-1.3659600019455, -0.092579491436481, -1.6386258602142, 0.16349516808987, -1.5712949037552, -0.030957356095314],
               [-1.1422111988068, -0.19559998810291, -1.7871555089951, 0.41443687677383, -1.5709083080292, -0.42891606688499],
               [-0.98222839832306, -0.20063918828964, -1.5140337944031, 0.14717110991478, -1.5707685947418, -0.76299875974655]]
    s1 = 0
    s2 = 0.25363121457115
    s3 = 0.16919199209086 + s2
    s4 = 0.16907819228188 + s3
    s5 = 0.22339127326454 + s4
    ss = [s1, s2, s3, s4, s5]
    return (
        ss,
        way_pts,
        [10] * dof,
        [30] * dof,
        # [3.92, 2.61, 2.85, 3.92, 3.02, 6.58],
        # [19.7, 16.8, 20.7, 20.9, 23.7, 33.5],
        # [200] * dof,
    )
ss, way_pts, vlims, alims = generate_new_problem()
begin_t = time.time()
################################################################################
# Define the geometric path and two constraints.
path = ta.SplineInterpolator(ss, way_pts)
pc_vel = constraint.JointVelocityConstraint(vlims)
pc_acc = constraint.JointAccelerationConstraint(alims)

################################################################################
# We solve the parametrization problem using the
# `ParametrizeConstAccel` parametrizer. This parametrizer is the
# classical solution, guarantee constraint and boundary conditions
# satisfaction.
instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
jnt_traj = instance.compute_trajectory()
duration = jnt_traj.duration
print('duration: ', duration)
################################################################################
# The output trajectory is an instance of
# :class:`toppra.interpolator.AbstractGeometricPath`.
ts_sample = np.linspace(0, duration, 100)
qs_sample = jnt_traj(ts_sample)

np.savetxt('../files/q.txt', qs_sample)

end_t = time.time()
compute_time = end_t - begin_t
print("Computation: ", compute_time)

qds_sample = jnt_traj(ts_sample, 1)
qdds_sample = jnt_traj(ts_sample, 2)
qdds_func = CubicSpline(ts_sample, qdds_sample)
qddds_func = qdds_func.derivative()
jerk_sample = qddds_func(ts_sample)

fig, axs = plt.subplots(4, 1, sharex=True)
for i in range(path.dof):
    # plot the i-th joint trajectory
    axs[0].plot(ts_sample, qs_sample[:, i], c="C{:d}".format(i))
    axs[1].plot(ts_sample, qds_sample[:, i], c="C{:d}".format(i))
    axs[2].plot(ts_sample, qdds_sample[:, i], c="C{:d}".format(i))
    axs[3].plot(ts_sample, jerk_sample[:, i], c="C{:d}".format(i))
axs[3].set_xlabel("Time (s)")
axs[0].set_ylabel("Position (rad)")
axs[1].set_ylabel("Velocity (rad/s)")
axs[2].set_ylabel("Acceleration (rad/s2)")
axs[3].set_ylabel("Jerk (rad/s3)")
plt.show()


################################################################################
# Optionally, we can inspect the output.
# instance.compute_reachable_sets(0, 0)
# instance.inspect()
