import numpy as np
import yaml
import math
from scipy.interpolate import CubicSpline


def calculate_ss(positions):
    ss = [0]
    l = len(positions)
    for i in range(1, l):
        sum = 0
        for j in range(6):
            sum += (positions[i][j]-positions[i-1][j])**2

        ss.append(ss[-1] + math.sqrt(sum))

    return ss


global_file = '../files/global_traj/global_traj.yaml'
combine_file = '../files/combine_traj/combine_traj.yaml'

fs1 = open(global_file, 'r')
data1 = yaml.load(fs1)
keys1 = sorted(data1.keys())

positions1 = []
for key in keys1:
    positions1.append(data1[key]['positions'])
ss1 = calculate_ss(positions1)
fs1.close()
func1 = CubicSpline(ss1, positions1, bc_type='not-a-knot')
dfunc1 = func1.derivative()
# print(dfunc1(ss1)[20, :])

fs2 = open(combine_file, 'r')
data2 = yaml.load(fs2)
keys2 = sorted(data2.keys())

point0 = [0.00016650317045385776, -0.7856533177961591, -2.5638997105836836e-05, -2.3559484214613482,
          -1.3939607426571854e-05, 1.5717442866382312, 0.7854100204066059]

positions2 = [point0]
for key in keys2:
    positions2.append(data2[key]['point_1']['positions'])
ss2 = calculate_ss(positions2)
fs2.close()

func2 = CubicSpline(ss2, positions2, bc_type='not-a-knot')
dfunc2 = func2.derivative()
# print(dfunc2(ss2)[20, :])
print(positions2)
print(ss2)

positions3 = positions2[20:]
ss3 = ss2[20:]
func3 = CubicSpline(ss3, positions3, bc_type='not-a-knot')
dfunc3 = func3.derivative()
# print(dfunc3(ss3)[0, :])