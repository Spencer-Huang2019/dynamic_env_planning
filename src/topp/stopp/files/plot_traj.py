import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.interpolate import CubicSpline

qs_sample = np.loadtxt('H:\\1-project\S-TOPP\S-TOPP_origin_v2\\files\q.txt')
ts_sample = np.loadtxt('H:\\1-project\S-TOPP\S-TOPP_origin_v2\\files\\ts.txt')
print(ts_sample)

qs_func = CubicSpline(ts_sample, qs_sample)
qds_func = qs_func.derivative()
qds_sample = qds_func(ts_sample)

qds_func = CubicSpline(ts_sample, qds_sample)
qdds_func = qs_func.derivative()
qdds_sample = qdds_func(ts_sample)

qdds_func = CubicSpline(ts_sample, qdds_sample)
qddds_func = qdds_func.derivative()
qddds_sample = qddds_func(ts_sample)

fig, axs = plt.subplots(4, 1, sharex=True)
for i in range(6):
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