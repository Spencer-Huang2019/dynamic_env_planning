# -*- coding: UTF-8 -*-
"""
    n_sample 很少，通过上一个u选取下一个x的方式，速度慢，但是效果更好，需要看看为什么这样做有效。
"""
import math
import time

import numpy as np
from math import *
import logging
import random
import matplotlib.pyplot as plt
# import plot_graph
import sympy

from ..algorithm import ParameterizationAlgorithm, ParameterizationReturnCode
from scipy.interpolate import CubicSpline
import sympy as sp

logger = logging.getLogger(__name__)


class DP3Algorithm(ParameterizationAlgorithm):
# class DP3Algorithm():
    """

    """
    class Node:
        def __init__(self, x):
            self.x = x
            self.next = None
            self.cost = None
            self.params = None
            self.zsk = None
            self.carry = []
            self.kz0 = None

    def __init__(
            self, constraint_list, path, gridpoints=None, n_s=5, n_z=100, xs_max=None, parametrizer=None, **kwargs
    ):
        super(DP3Algorithm, self).__init__(
            constraint_list, path, gridpoints=gridpoints, parametrizer=parametrizer, **kwargs
        )
        gridpoints = self.problem_data.gridpoints

        self.qs = path(gridpoints, 1)
        self.qss = path(gridpoints, 2)
        self.qsss = path(gridpoints, 3)
        self.dof = path.dof
        self.N = len(gridpoints)
        if self.N % n_s != 0:
            self.p = self.N // n_s
            if self.N % self.p == 0:
                self.n_s = int(self.N / self.p) + 1
            else:
                self.n_s = int(self.N / self.p) + 2
        else:
            self.p = self.N // n_s
            self.n_s = n_s
        self.n_z = n_z
        self.ss = self.ss = gridpoints
        xs_min = np.zeros(self.N).reshape(1, -1)
        xs_range = np.insert(xs_max, 0, values=xs_min, axis=1)

        self.K = xs_range

        self.Sch, self.Kch, self.Qsch, self.Qssch, self.Qsssch = self.get_Sch_Kch_set()
        # print(self.Kch)

        self.constraints = constraint_list
        self.xs_start = None
        self.xs_end = None
        self.ts = None

    def get_Sch_Kch_set(self):
        Sch = [[]]
        Kch = [[]]
        Qsch = [[]]
        Qssch = [[]]
        Qsssch = [[]]
        for i in range(self.n_s - 2):
            if i != 0:
                Sch[i].append(self.ss[self.p*i - 1])
                Kch[i].append(self.K[self.p*i - 1])
                Qsch[i].append(self.qs[self.p * i - 1])
                Qssch[i].append(self.qss[self.p * i - 1])
                Qsssch[i].append(self.qsss[self.p * i - 1])
            for j in range(self.p):
                Sch[i].append(self.ss[self.p*i + j])
                Kch[i].append(self.K[self.p * i + j])
                Qsch[i].append(self.qs[self.p * i + j])
                Qssch[i].append(self.qss[self.p * i + j])
                Qsssch[i].append(self.qsss[self.p * i + j])
            Sch.append([])
            Kch.append([])
            Qsch.append([])
            Qssch.append([])
            Qsssch.append([])
        # print(len(self.ss[self.n_s * p:]))
        Sch[self.n_s - 2].extend(self.ss[(self.n_s - 2) * self.p - 1:])
        Kch[self.n_s - 2].extend(self.K[(self.n_s - 2) * self.p - 1:])
        Qsch[self.n_s - 2].extend(self.qs[(self.n_s - 2) * self.p - 1:])
        Qssch[self.n_s - 2].extend(self.qss[(self.n_s - 2) * self.p - 1:])
        Qsssch[self.n_s - 2].extend(self.qsss[(self.n_s - 2) * self.p - 1:])

        return Sch, Kch, Qsch, Qssch, Qsssch

    def compute_parameterization(self, sd_start, sd_end, return_data=False):

        self.xs_start = self.Node(sd_start ** 2)
        self.xs_end = self.Node(sd_end ** 2)

        valid_nodes = None
        # backward search
        for k in range(self.n_s-2, 1, -1):
            # print("k = {}".format(k))
            discrete_nodes = self.discrete_x(self.Kch[k][0])
            if k == self.n_s-2:
                valid_nodes = self.backward_search_endnode(discrete_nodes, self.Sch[k], self.Qsch[k], self.Qssch[k], self.Qsssch[k])
            else:
                if len(valid_nodes) > 0:
                    valid_nodes = self.backward_search_middle(valid_nodes, discrete_nodes, self.Sch[k], self.Qsch[k], self.Qssch[k], self.Qsssch[k])
                else:
                    print("Solution is infeasible!!!")
                    return False
        if len(valid_nodes) > 0:
            discrete_nodes = self.discrete_x(self.Kch[1][0])
            if self.backward_search_start(valid_nodes, discrete_nodes, self.Sch[:2], self.Qsch[:2], self.Qssch[:2], self.Qsssch[:2]):
                print("Finish the backward searching!")
        else:
            print("Solution is infeasible!!!")
            return False

        # xs_node = self.optimal_solution()
        # xs_final, self.ts = self.refine(xs_node)

        xs_final, self.ts = self.optimal_solution()
        # print(len(xs_final))

        vs = np.sqrt(np.array(xs_final))
        qd = np.multiply(self.qs, vs[:, np.newaxis])

        qs = self.path(self.ss)
        # self.draw_static(xs_final, np.linspace(0, 836, 837), self.K)

        return qs, qd, self.ts

        # self.draw_static_set(xs_set, self.Sch, self.Kch)
        # self.draw_traj(xs_final)

    def discrete_x(self, K):

        x_lower = K[0]
        x_upper = K[1]
        delta = x_upper - x_lower

        nodes = []
        for j in range(0, self.n_z):
            x = x_lower + j * delta / (self.n_z-1)
            nodes.append(self.Node(x))

        return nodes

    def backward_search_endnode(self, d_nodes, Sch, Qs, Qss, Qsss):
        valid_nodes = []
        # print(len(Sch))
        for j in range(1, self.n_z):

            valid = True
            xk = d_nodes[j].x
            carry = [[xk, 0]]
            delta = Sch[-1] - Sch[0]
            cost, params = self.interpolator_2(xk, self.xs_end.x, delta)
            y = 0
            for s in Sch:
                kz = (s - Sch[-1]) / delta
                x, zs, zss = self.calc_z_deri(2, kz, params, delta)
                sdd = zs * 0.5
                sddd = zss * sqrt(x) * 0.5
                constraint_1 = self.order_1_constraint(Qs[y], x)
                constraint_2 = self.order_2_constraint(Qs[y], Qss[y], x, sdd)
                constraint_3 = self.order_3_constraint(Qs[y], Qss[y], Qsss[y], x, sdd, sddd)

                if not(constraint_1 and constraint_2 and constraint_3):
                    valid = False
                    break

                if y > 0:
                    cost_ = self.calc_cost(kz, params, delta)
                    if cost_ < 0:
                        valid = False
                        break

                    carry.append([x, cost_])

                y += 1

            if valid:
                d_nodes[j].next = self.xs_end
                d_nodes[j].cost = cost
                # print(cost)
                d_nodes[j].carry = carry
                # print(len(carry))
                d_nodes[j].params = params
                d_nodes[j].zsk = (self.xs_end.x - xk) / delta
                valid_nodes.append(d_nodes[j])

        return valid_nodes

    def backward_search_middle(self, last_nodes, d_nodes, Sch, Qs, Qss, Qsss):
        valid_nodes = []
        for j in range(1, self.n_z):
            xk = d_nodes[j].x
            valid = False
            Jmin = 10000
            for parent_node in last_nodes:
                carry = [[xk, 0]]
                L = self.calc_search_range(xk, Sch, Qs[0], Qss[0])
                xk1 = parent_node.x
                if L[0] <= xk1 <= L[1]:
                    break_out = False
                    zsk = parent_node.zsk
                    delta = Sch[-1]-Sch[0]
                    cost, params = self.interpolator_3(xk, xk1, delta, zsk)
                    y = 0
                    for s in Sch:
                        kz = (s - Sch[-1]) / (delta)
                        x, zs, zss = self.calc_z_deri(3, kz, params, delta)
                        if x <= 0.001:
                            break_out = True
                            break
                        sdd = zs * 0.5
                        sddd = zss * sqrt(x) * 0.5
                        constraint_1 = self.order_1_constraint(Qs[y], x)
                        constraint_2 = self.order_2_constraint(Qs[y], Qss[y], x, sdd)
                        constraint_3 = self.order_3_constraint(Qs[y], Qss[y], Qsss[y], x, sdd, sddd)

                        if not (constraint_1 and constraint_2 and constraint_3):
                            break_out = True
                            break

                        if y > 0:
                            cost_ = self.calc_cost(kz, params, delta)
                            if cost_ < 0:
                                break_out = True
                                break
                            carry.append([x, cost_])
                        y += 1
                    if not break_out:
                        valid = True
                        if cost < Jmin:
                            Jmin = cost
                            parent = parent_node
                            params_final = params
                            carry_final = carry
                            zsk_final = 2 * (params[2] - params[1] + params[0]) * (params[1] - 2 * params[2]) / (Sch[-1] - Sch[0])
            if valid:
                d_nodes[j].next = parent
                d_nodes[j].params = params_final
                d_nodes[j].zsk = zsk_final
                d_nodes[j].cost = Jmin
                d_nodes[j].carry = carry_final
                valid_nodes.append(d_nodes[j])

        return valid_nodes

    def calc_search_range(self, x, sch, qs, qss):
        max_slop = 100000
        min_slop = -100000
        constraint = self.constraints[1]

        # calc x min and max slope
        for k in range(self.dof):
            l_zsijk = 2*(constraint[k][0] - qss[k] * x) / qs[k]
            u_zsijk = 2*(constraint[k][1] - qss[k] * x) / qs[k]
            max_slop = min(max_slop, max(l_zsijk, u_zsijk))
            min_slop = max(min_slop, min(l_zsijk, u_zsijk))

        # calc search range() range on parent nodes
        lower = min_slop * (sch[-1] - sch[0]) + x
        upper = max_slop * (sch[-1] - sch[0]) + x

        return [lower, upper]

    def backward_search_start(self, last_nodes, d_nodes_2, Sch, Qs, Qss, Qsss):
        Sch1 = Sch[0]
        Sch2 = Sch[1]
        xk = self.xs_start.x
        valid = False
        Jmin_total = 1000
        valid_node3 = []
        # Jmin2 = 1000
        # parent_1 = None
        # parent_2 = None
        for parent_node in d_nodes_2[1:]:
            # print(parent_node.x)
            J = self.calc_search_range(xk, Sch1, Qs[0][0], Qss[0][1])
            xk1 = parent_node.x
            if xk1 == 0:
                continue
            if J[0] <= xk1 <= J[1]:
                break_out = False
                cost, params = self.interpolator_2(xk, xk1, Sch1[-1] - Sch1[0])
                y = 0
                carry_1 = [[xk, 0]]
                for s in Sch1:
                    kz = (s - Sch1[-1]) / (Sch1[-1] - Sch1[0])
                    x, zs, zss = self.calc_z_deri(2, kz, params, Sch1[-1] - Sch1[0])
                    if y > 0 and x < 0.001:
                        break_out = True
                        break
                    sdd = zs * 0.5
                    sddd = zss * sqrt(x) * 0.5
                    constraint_1 = self.order_1_constraint(Qs[0][y], x)
                    constraint_2 = self.order_2_constraint(Qs[0][y], Qss[0][y], x, sdd)
                    constraint_3 = self.order_3_constraint(Qs[0][y], Qss[0][y], Qsss[0][y], x, sdd, sddd)

                    if not (constraint_1 and constraint_2 and constraint_3):
                        break_out = True
                        break

                    if y > 0:
                        cost_ = self.calc_cost(kz, params, Sch1[-1] - Sch1[0])
                        if cost_ < 0:
                            break_out = True
                            break
                        carry_1.append([x, cost_])
                    y += 1
                if not break_out:
                    Jmin2 = 1000
                    for node_3 in last_nodes:
                        xk_3 = xk1
                        xk1_3 = node_3.x
                        L = self.calc_search_range(xk_3, Sch2, Qs[1][0], Qss[1][1])
                        if L[0] <= xk1_3 <= L[1]:
                            carry_2 = [[xk_3, 0]]
                            break_out3 = False
                            # cost3, params3 = self.interpolator_3(xk_3, xk1_3, Sch2[-1] - Sch2[0], node_3.zsk)
                            cost3, params3, isReal, kz0 = self.interpolator_4(xk_3, xk1_3, Sch2[-1]-Sch2[0], (xk1 - xk)/(Sch1[-1] - Sch1[0]), node_3.zsk)
                            if not isReal or cost3 < 0:
                                continue
                            y = 0
                            for s in Sch2:
                                kz = (s - Sch2[-1]) / (Sch2[-1] - Sch2[0])
                                x, zs, zss = self.calc_z_deri(4, kz, params3, Sch2[-1] - Sch2[0])
                                if x < 0.01:
                                    break_out3 = True
                                    break
                                sdd = zs * 0.5
                                sddd = zss * sqrt(x) * 0.5
                                constraint_1 = self.order_1_constraint(Qs[1][y], x)
                                constraint_2 = self.order_2_constraint(Qs[1][y], Qss[1][y], x, sdd)
                                constraint_3 = self.order_3_constraint(Qs[1][y], Qss[1][y], Qsss[1][y], x, sdd, sddd)

                                if not (constraint_1 and constraint_2 and constraint_3):
                                    break_out3 = True
                                    break
                                if y > 0:
                                    cost_ = self.calc_cost(kz, params3, Sch2[-1] - Sch2[0], kz0=kz0)
                                    if cost_ < 0:
                                        break_out3 = True
                                        break
                                    carry_2.append([x, cost_])
                                y += 1

                            if not break_out3:
                                valid = True
                                valid_node3.append(node_3)
                                if Jmin2 > cost3:
                                    Jmin2 = cost3
                                    parent_2 = node_3
                                    params2_final = params3
                                    zsk2_final = 2 * params3[0] * params3[1] / (Sch2[-1] - Sch2[0])
                                    carry2_final = carry_2
                                    kz0_final = kz0
                    if valid:
                        parent_node.next = parent_2
                        parent_node.params = params2_final
                        parent_node.carry = carry2_final
                        parent_node.zsk = zsk2_final
                        parent_node.kz0 = kz0_final
                        parent_node.cost = Jmin2

                if valid and Jmin_total > (cost + Jmin2) and parent_node.next is not None:
                    Jmin_total = cost + Jmin2
                    cost1 = cost
                    parent_1 = parent_node
                    params1_final = params
                    carry1_final = carry_1
                    zsk1_final = (parent_node.x - xk) / (Sch1[-1] - Sch1[0])

        if len(valid_node3) > 0:
            self.xs_start.next = parent_1
            self.xs_start.cost = cost1
            self.xs_start.params = params1_final
            self.xs_start.zsk = zsk1_final
            self.xs_start.carry = carry1_final
            # print(parent_1.x)
            # print(parent_1.next.x)
            # print(parent_1.zsk)
            # print(parent_1.next.zsk)
            # print(parent_1.params)
            # print(Sch2[-1] - Sch2[0])
            # print(parent_1.carry)
            return True
        else:
            print("Solution is infeasible!!!")
            return False

    def calc_z_deri(self, order, kz, params, delta):
        if order == 2:
            x = params[1] * kz + params[0]
            zs = params[1] / delta
            zss = 0
        elif order == 3:
            x = (params[2] * kz**2 + params[1]*kz + params[0])**2
            zs = 2 * sqrt(x) * (2*params[2]*kz + params[1]) / delta
            zss = 2 * (((2 * params[2] * kz + params[1])/delta) ** 2 + 2 * params[2] * sqrt(x) / (delta**2))
        else:
            x = (params[3] * kz ** 3 + params[2] * kz **2 + params[1] * kz + params[0]) ** 2
            zs = 2 * sqrt(x) * (3 * params[3] * kz **2 + 2 * params[2] * kz + params[1]) / delta
            zss = 2 * (((3 * params[3] * kz**2 + 2 * params[2] * kz + params[1])/delta) ** 2 + (6 * params[3] * kz + 2 * params[2]) * sqrt(x) / (delta**2))

        return x, zs, zss

    def calc_cost(self, kz, params, delta, kz0=None):
        order = len(params)
        if order == 2:
            if params[1] == 0:
                cost = (kz + 1) / sqrt(params[0])
            else:
                cost = 2*(sqrt(params[1] * kz + params[0]) - sqrt(params[0] - params[1])) / params[1]
        elif order == 3:
            cost = self.seg_integrate(kz, params) - self.seg_integrate(-1, params)
        else:
            cost = self.seg_integrate4(kz0, kz, params) - self.seg_integrate4(kz0, -1, params)

        return cost * delta

    def seg_integrate(self, kz, params):
        c = 4 * params[0] * params[2] - params[1] ** 2
        if c == 0:
            integrate = -2 / (2 * params[2] * kz + params[1])
        elif c > 0:
            integrate = 2 * atan((2 * params[2] * kz + params[1]) / sqrt(c)) / sqrt(c)
        else:
            integrate = log(
                abs((2 * params[2] * kz + params[1] - sqrt(-c)) / (2 * params[2] * kz + params[1] + sqrt(-c)))) / sqrt(
                -c)

        return integrate

    def seg_integrate4(self, kz0, kz, params):

        c32 = params[3]
        c31 = params[2] + params[3] * kz0
        c30 = params[1] + params[2] * kz0 + params[3] * kz0 **2
        B1 = c32 * kz **2 + c31 * kz + c30
        B2 = c32 * kz0 **2 + c31 * kz0 + c30
        A1 = 1/B2
        A2 = (-c31 - c32 * kz0) / B2
        A3 = -c32 / B2
        integrate1 = A1 * log(abs(kz - kz0))
        integrate2 = A2 * self.seg_integrate(kz, [c30, c31, c32])
        integrate3 = A3 * (log(abs(B1)) / (2 * c32) - c31 * self.seg_integrate(kz, [c30, c31, c32]) / (2*c32))

        return integrate1 + integrate2 + integrate3

    # def calc_cost(self, xk, xk1, delta):
    #     return 2 * delta / (sqrt(xk) + sqrt(xk1))

    def interpolator_2(self, xk, xk1, delta):
        c0 = xk1
        c1 = c0 - xk
        params = [c0, c1]
        cost = self.calc_cost(0, params, delta)
        # cost = self.calc_cost(xk, xk1, delta)

        return cost * delta, params

    def interpolator_3(self, xk, xk1, delta, zsk):
        c0 = sqrt(xk1)
        c1 = 0.5 * zsk * delta / c0
        c2 = sqrt(xk) - c0 + c1
        params = [c0, c1, c2]
        cost = self.seg_integrate(0, params) - self.seg_integrate(-1, params)
        # print(cost)

        return cost * delta, params

    def interpolator_4(self, xk, xk1, delta, zsk, zsk1):
        c0 = sqrt(xk1)
        c1 = 0.5 * zsk1 * delta / c0
        c2 = 0.5 * zsk * delta / sqrt(xk) + 3 * sqrt(xk) + 2 * c1 - 3 * c0
        c3 = c2 - c1 + c0 - sqrt(xk)
        params = [c0, c1, c2, c3]
        cost = None
        x = sp.Symbol('x')
        f = c3 * x ** 3 + c2 * x ** 2 + c1 * x + c0
        kz0_list = sp.solve(f)
        kz0 = kz0_list[0]
        if isinstance(kz0, sp.core.numbers.Float):
            real = True
            cost = self.seg_integrate4(kz0, 0, params) - self.seg_integrate4(kz0, -1, params)
        else:
            real = False

        return cost, params, real, kz0

    def order_1_constraint(self, qs, x):
        constraint = self.constraints[0]
        for k in range(self.dof):
            qd = qs[k] * sqrt(x)
            if constraint[k][0] >= qd or qd >= constraint[k][1]:
                return False
        return True

    def order_2_constraint(self, qs, qss, x, sdd):
        constraint = self.constraints[1]
        for k in range(self.dof):
            qdd = qs[k] * sdd + qss[k] * x
            if constraint[k][0] >= qdd or qdd >= constraint[k][1]:
                return False
        return True

    def order_3_constraint(self, qs, qss, qsss, x, sdd, sddd):
        constraint = self.constraints[2]
        for k in range(self.dof):
            qddd = qs[k] * sddd + 3 * qss[k] * sdd * x + qsss[k] * x * sqrt(x)
            if constraint[k][0] >= qddd or qddd >= constraint[k][1]:
                return False
        return True

    def optimal_solution(self):
        node = self.xs_start
        xs_final = [self.xs_start.x]
        ts = [0]
        xs_node = [node]
        while node.next is not None:
            xs_node.append(node.next)
            node = node.next
        # print([node.x for node in xs_node])
        l = len(xs_node)
        # print(l)
        ts_last = 0
        for i in range(l-1):
            carry = xs_node[i].carry
            # print(carry)
            y = 0
            for c in carry:
                if y == 0:
                    y += 1
                    continue
                xs_final.append(c[0])
                ts.append(c[1] + ts_last)
                y += 1
            ts_last = ts[-1]
        # print(ts[-1])
        return xs_final, ts

    def refine(self, xs_node):
        ts = [0]
        xs = [0]
        i = 0
        for sch in self.Sch:
            delta = sch[-1] - sch[0]
            j = 0
            ts_cal = 0
            # print('zs at end-point: {}'.format(xs_node[i].zsk))
            for s in sch:
                if j == 0:
                    j += 1
                    continue
                params = xs_node[i].params
                order = len(params)
                kz = (s - sch[-1]) / delta
                x, zs, _ = self.calc_z_deri(order, kz, params, delta)
                # if s == sch[-1]:
                    # print('zs from inter: {}'.format(zs))
                cost = self.calc_cost(kz, params, s - sch[0])
                ts.append(ts[-1] + cost - ts_cal)
                # cost = self.calc_cost(xs_node[i].x, x, s - sch[j-1])
                # ts.append(ts[-1] + cost)
                ts_cal = cost
                xs.append(x)
                j += 1
            i += 1
        print(ts[-1])
        print(xs)
        return xs, ts

    def draw_static(self, xs, ss, K):
        plt.rcParams['figure.figsize'] = (10, 6.5)
        plt.rc('text', usetex=True)
        plt.rc('font', family='Times New Roman')

        for y in range(self.N - 1):
            plt.plot([ss[y], ss[y+1]], [xs[y], xs[y+1]], '-g', zorder=1, linewidth=2)
            plt.plot([ss[y], ss[y+1]], [K[y][1], K[y+1][1]], 'grey', linestyle='--', zorder=2, linewidth=2)

        font_size = 20
        plt.xlabel('$s$ index', fontdict={"size": font_size}, labelpad=15)
        plt.ylabel('$x$', fontdict={"size": font_size}, labelpad=15, rotation=0)
        bwith = 1
        ax = plt.gca()
        ax.spines['bottom'].set_linewidth(bwith)
        ax.spines['left'].set_linewidth(bwith)
        ax.spines['top'].set_linewidth(bwith)
        ax.spines['right'].set_linewidth(bwith)
        ax.tick_params(which="major", length=6, width=1.0)

        plt.xticks(fontsize=font_size-2, usetex=True)
        plt.yticks(fontsize=font_size-2, usetex=True)
        plt.legend(["Edge", "$\mathcal{L}$"], ncol=1, prop={"family": "Times New Roman", "size": font_size},
                   bbox_to_anchor=(0.25, 0.9))
        plt.gcf().subplots_adjust(bottom=0.25)
        # plt.savefig(
        #     "H:/2-writing/stopp_v8/picture/traj_sample/stopp/keep_u/e7_" + time.strftime('%Y%m%d_%H-%M-%S') + ".pdf")
        plt.show()

    def draw_static_set(self, xs, ss, K):
        plt.rcParams['figure.figsize'] = (10, 6.5)
        plt.rc('text', usetex=True)
        plt.rc('font', family='Times New Roman')

        for y in range(self.n_s - 1):
            plt.plot([ss[y][0], ss[y][-1]], [xs[y], xs[y+1]], '-g', zorder=1, linewidth=2)
            plt.plot([ss[y][0], ss[y][-1]], [K[y][0], K[y][-1]], 'grey', linestyle='--', zorder=2, linewidth=2)

        font_size = 20
        plt.xlabel('$s$ index', fontdict={"size": font_size}, labelpad=15)
        plt.ylabel('$x$', fontdict={"size": font_size}, labelpad=15, rotation=0)
        bwith = 1
        ax = plt.gca()
        ax.spines['bottom'].set_linewidth(bwith)
        ax.spines['left'].set_linewidth(bwith)
        ax.spines['top'].set_linewidth(bwith)
        ax.spines['right'].set_linewidth(bwith)
        ax.tick_params(which="major", length=6, width=1.0)

        plt.xticks(fontsize=font_size-2, usetex=True)
        plt.yticks(fontsize=font_size-2, usetex=True)
        plt.legend(["Edge", "$\mathcal{L}$"], ncol=1, prop={"family": "Times New Roman", "size": font_size},
                   bbox_to_anchor=(0.25, 0.9))
        plt.gcf().subplots_adjust(bottom=0.25)
        # plt.savefig(
        #     "H:/2-writing/stopp_v8/picture/traj_sample/stopp/keep_u/e7_" + time.strftime('%Y%m%d_%H-%M-%S') + ".pdf")
        plt.show()

    def draw_velocity(self, xs):
        vs = np.sqrt(np.array(xs))
        qd = np.multiply(self.qs, vs[:, np.newaxis])
        for i in range(self.dof):
            # plot the i-th joint trajectory
            plt.plot(self.ss, qd[:, i], c="C{:d}".format(i))
        plt.show()

    def draw_traj(self, xs):
        vs = np.sqrt(np.array(xs))
        qd = np.multiply(self.qs, vs[:, np.newaxis])

        qds_func = CubicSpline(self.ts, qd)
        qdds_func = qds_func.derivative()
        qdd = np.multiply(qdds_func(self.ts), vs[:, np.newaxis])

        qdds_func = CubicSpline(self.ts, qdd)
        qddds_func = qdds_func.derivative()
        qddd = np.multiply(qddds_func(self.ts), vs[:, np.newaxis])

        fig, axs = plt.subplots(3, 1, sharex=True)
        for i in range(self.dof):
            # plot the i-th joint trajectory
            # j = 0
            # for q3 in qddd[:, i]:
            #     if q3 > 500 or q3 < -500:
            #         print('joint_{}'.format(i))
            #         print('s_index = {}'.format(j))
            #     j += 1

            axs[0].plot(self.ts, qd[:, i], c="C{:d}".format(i))
            axs[1].plot(self.ts, qdd[:, i], c="C{:d}".format(i))
            axs[2].plot(self.ts, qddd[:, i], c="C{:d}".format(i))
        axs[2].set_xlabel("Time(s)")
        axs[0].set_ylabel("Velocity (rad/s)")
        axs[1].set_ylabel("Acceleration (rad/s2)")
        axs[2].set_ylabel("Jerk (rad/s3)")
        plt.show()

if __name__ == '__main__':
    ps = np.loadtxt('../../examples/ps.txt')
    pss = np.loadtxt('../../examples/pss.txt')
    psss = np.loadtxt('../../examples/psss.txt')
    xs_max = np.loadtxt('../../examples/xs.txt').reshape(-1, 1)
    ss = np.loadtxt('../../examples/ss.txt')

    N = ps.shape[0]
    xs_min = np.zeros(N).reshape(1, -1)
    xs_range = np.insert(xs_max, 0, values=xs_min, axis=1)
    # xs_range = np.loadtxt('D:\\research\projects\TOPP\jerk\sample_based\sample_based_v1\examples\\K.txt').reshape(-1, 2)

    # case 1
    vlim = [[-10, 10], [-10, 10], [-10, 10], [-10, 10], [-10, 10], [-10, 10]]
    alim = [[-30, 30], [-30, 30], [-30, 30], [-30, 30], [-30, 30], [-30, 30]]
    j_min = -500
    j_max = 500

    # case 2
    # vlim = [[-3.92, 3.92], [-2.61, 2.61], [-2.85, 2.85], [-3.92, 3.92], [-3.02, 3.02], [-6.58, 6.58]]
    # alim = [[-19.7, 19.7], [-16.8, 16.8], [-20.7,  20.], [-20.9, 20.9], [-23.7, 23.7], [-33.5, 33.5]]
    # j_min = -250
    # j_max = 250

    # *****************************************************
    jlim = [[j_min, j_max], [j_min, j_max], [j_min, j_max], [j_min, j_max], [j_min, j_max], [j_min, j_max]]
    constraints_list = [vlim, alim, jlim]
    start_t = time.time()
    DP3Planner = DP3Algorithm(constraints_list, ps, pss, psss, ss, xs_max=xs_range)
    DP3Planner.compute_parameterization(0, 0)

    """ **********************************************************************************************
                                    test functions
    # ****************************************************************************************************"""


    """ **************************************************************************************************** """

    # node_list = SamplePlanner.planning()
    # xs = [node_list[0].x]
    # sdd = [node_list[0].sdd]
    # sddd = [node_list[0].sddd]
    # ts = [node_list[0].cost]
    # for node in node_list[1:]:
    #     xs.append(node.x)
    #     sdd.append(node.sdd)
    #     sddd.append(node.sddd)
    #     ts.append(ts[-1] + node.cost)
    #
    # xs = np.array(xs)
    # sdd = np.array(sdd)
    # sddd = np.array(sddd)
    #
    # np.savetxt('./sample_xs.txt', xs)
    # np.savetxt('./sample_sdd.txt', sdd)
    # # print(xs)
    # # print('Duration: ', ts[-1])
    # #
    # import matplotlib.pyplot as plt
    # plt.plot(xs_range, '--', c='red', label='xs bound')
    # plt.plot(xs, c='blue', label='xs profile')
    # plt.legend()
    # plt.xlabel('s')
    # plt.ylabel('sd square')
    # plt.show()
    # #
