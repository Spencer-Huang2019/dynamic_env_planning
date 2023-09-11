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
    way_pts = [
        [-1.8228123188019, -0.22999736666679, -1.4806070327759, 0.1408361941576, -1.570325255394, 0.2526221871376],
        [-1.5998414754868, -0.19636431336403, -1.7848707437515, 0.41382598876953, -1.5719540119171, 0.029550120234489],
        [-1.3651517629623, -0.093969941139221, -1.6383001804352, 0.16477064788342, -1.5716984272003, -0.20556211471558],
        [-1.1404975652695, -0.19580484926701, -1.7863637208939, 0.41435623168945, -1.5715783834457, -0.42988502979279],
        [-0.98446577787399, -0.19861748814583, -1.5143086910248, 0.14629447460175, -1.5709903240204, -0.15106952190399],
        [-0.84857839345932, -0.36521553993225, -1.560854434967, 0.35915490984917, -1.5710736513138, -0.72189337015152]]
    return (
        # np.linspace(0, 1, N_samples),
        [0, 0.51740778202715, 0.96946219228068, 1.4114762388342, 1.909435381494, 2.5571515507738],
        way_pts,
        [1] * dof,
        [10] * dof,
        # [3.92, 2.61, 2.85, 3.92, 3.02, 6.58],
        # [19.7, 16.8, 20.7, 20.9, 23.7, 33.5],
        [200] * dof,
    )
ss, way_pts, vlims, alims, jlims = generate_new_problem()
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
instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel", gridpt_max_err_threshold=1e-4, gridpt_min_nb_points=200)
_, sd_ra, _ = instance.compute_parameterization(0, 0, return_data=True)
instance_j = algo.DP3Algorithm([pc_vel.formal, pc_acc.formal, pc_jerk.formal], path, n_s=10, n_z=5,
                                    xs_max=(sd_ra ** 2).reshape(-1, 1), parametrizer="ParametrizeConstAccel", gridpt_max_err_threshold=1e-4, gridpt_min_nb_points=200)
qs, qds, ts = instance_j.compute_trajectory_jerk_DP3()
################################################################################
# The output trajectory is an instance of
# :class:`toppra.interpolator.AbstractGeometricPath`.
end_t = time.time()
compute_time = end_t - begin_t
duration = float(ts[-1])
print('duration: ', duration)
print("Computation: ", compute_time)

ts_sample = np.linspace(0, duration, 100)

qs_func = CubicSpline(ts, qs)
qs_sample = qs_func(ts_sample)

# qds_func = CubicSpline(ts, qds)
qds_func = qs_func.derivative()
qds_sample = qds_func(ts_sample)

qdds_func = qds_func.derivative()
qdds_sample = qdds_func(ts_sample)

qddds_func = qdds_func.derivative()
qddds_sample = qddds_func(ts_sample)

fig, axs = plt.subplots(4, 1, sharex=True)
for i in range(path.dof):
    # plot the i-th joint trajectory
    axs[0].plot(ts_sample, qs_sample[:, i], c="C{:d}".format(i))
    axs[1].plot(ts_sample, qds_sample[:, i], c="C{:d}".format(i))
    axs[2].plot(ts_sample, qdds_sample[:, i], c="C{:d}".format(i))
    axs[3].plot(ts_sample, qddds_sample[:, i], c="C{:d}".format(i))
axs[3].set_xlabel("Time (s)")
axs[0].set_ylabel("Path (rad)")
axs[1].set_ylabel("Velocity (rad/s)")
axs[2].set_ylabel("Acceleration (rad/s2)")
axs[3].set_ylabel("Jerk (rad/s3)")
plt.show()


################################################################################
# Optionally, we can inspect the output.
# instance.compute_reachable_sets(0, 0)
# instance.inspect()
