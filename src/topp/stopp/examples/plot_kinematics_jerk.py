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
# cd = "cd /home/spencer/projects/sample_based"
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
    dof = 7
    np.random.seed(seed)
    way_pts = [[-0.1830125073600466, 0.40226018810632574, -0.4177967213352571, -2.4301263698029496, -0.1389870735899615, 3.256897200061631, 0.878114300738498], [-0.19099756465836268, 0.45404587388093437, -0.4360106515638755, -2.4333596412174785, -0.14504568995633352, 3.3303594718893104, 0.8821562798763872], [-0.22535931247342378, 0.6680587020370178, -0.5102529946061782, -2.446830043139588, -0.17008115846095606, 3.6312488284984608, 0.8982561361832545]]
    # s1 = 0
    # s2 = 0.62502068218042
    # s3 = 1.3964821662243
    # s4 = 1.986230145641
    # s5 = 2.6300410972431
    # ss = [s1, s2, s3, s4, s5]
    return (
        # ss,
        [0.0, 0.0012196384999038153, 0.0218047788618526],
        # np.linspace(0, 1, N_samples),
        way_pts,
        [[-1.0875, 1.0875], [-1.0875, 1.0875], [-1.0875, 1.0875], [-1.0875, 1.0875], [-1.305, 1.305], [-1.305, 1.305], [-1.305, 1.305]],
        [[-1.875, 1.875], [-0.9375, 0.9375], [-1.25, 1.25], [-1.5625, 1.5625], [-1.875, 1.875], [-2.5, 2.5], [-2.5, 2.5]],
        # [1.2] * dof,
        # [10] * dof,
        # [3.92, 2.61, 2.85, 3.92, 3.02, 6.58],
        # [19.7, 16.8, 20.7, 20.9, 23.7, 33.5],
        [1000] * dof,
    )
ss, way_pts, vlims, alims, jlims = generate_new_problem()
init_path_vel = 0.01449861070785806
begin_t = time.time()
################################################################################
# Define the geometric path and two constraints.
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
duration = jnt_traj.duration
print('duration: ', duration)
################################################################################
# The output trajectory is an instance of
# :class:`toppra.interpolator.AbstractGeometricPath`.
ts_sample = np.linspace(0, duration, 100)
qs_sample = jnt_traj(ts_sample)

end_t = time.time()
compute_time = end_t - begin_t
print("Computation: ", compute_time)

qds_sample = jnt_traj(ts_sample, 1)
qdds_sample = jnt_traj(ts_sample, 2)
# qddds_sample = jnt_traj(ts_sample, 3)
qdds_func = CubicSpline(ts_sample, qdds_sample)
qddds_func = qdds_func.derivative()
qddds_sample = qddds_func(ts_sample)

import os
folder = '../files/s-topp_' + time.strftime('%Y%m%d_%H-%M-%S')
os.makedirs(folder)
np.savetxt('../files/' + folder + '/q.txt', qs_sample)
np.savetxt('../files/' + folder + '/qd.txt', qds_sample)
np.savetxt('../files/' + folder + '/qdd.txt', qdds_sample)
np.savetxt('../files/' + folder + '/qddd.txt', qddds_sample)
np.savetxt('../files/' + folder + '/ts.txt', ts_sample)

fig, axs = plt.subplots(4, 1, sharex=True)
for i in range(path.dof):
    # plot the i-th joint trajectory
    axs[0].plot(ts_sample, qs_sample[:, i], c="C{:d}".format(i))
    axs[1].plot(ts_sample, qds_sample[:, i], c="C{:d}".format(i))
    axs[2].plot(ts_sample, qdds_sample[:, i], c="C{:d}".format(i))
    axs[3].plot(ts_sample, qddds_sample[:, i], c="C{:d}".format(i))
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
