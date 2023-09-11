"""
    random
"""
import time

import numpy as np
from math import *
import logging
import random
import matplotlib.pyplot as plt
# import plot_graph
from ..algorithm import ParameterizationAlgorithm, ParameterizationReturnCode


logger = logging.getLogger(__name__)


class SamplingAlgorithm_random(ParameterizationAlgorithm):
    """

    """
    class Node:
        def __init__(self, x):
            self.x = x
            self.sdd = None
            self.sddd = None
            self.parent = None
            self.cost = 0.0

    def __init__(
            self, constraint_list, path, gridpoints=None, n_samples=100, ratio=1, xs_max=None, parametrizer=None, **kwargs
    ):
        super(SamplingAlgorithm_random, self).__init__(
            constraint_list, path, gridpoints=gridpoints, parametrizer=parametrizer, **kwargs
        )
        gridpoints = self.problem_data.gridpoints

        self.qs = path(gridpoints, 1)
        self.qss = path(gridpoints, 2)
        self.qsss = path(gridpoints, 3)
        self.dof = path.dof
        self.N = len(gridpoints)
        self.n_samples = n_samples
        self.ratio = ratio
        self.ss = gridpoints
        self.deltas = self.ss[1:] - self.ss[:-1]
        self.constraints = constraint_list

        xs_min = np.zeros(self.N).reshape(1, -1)
        xs_range = np.insert(xs_max, 0, values=xs_min, axis=1)

        self.K = xs_range
        self.xs_start = None
        self.xs_end = None

    def compute_parameterization(self, sd_start, sd_end, return_data=False):

        self.xs_start = self.Node(sd_start ** 2)
        self.xs_end = self.Node(sd_end ** 2)

        i = 1
        tries = 0
        MAX_TRIES = 30
        V_open = [self.xs_start]
        sample_collection = [V_open]

        while i < self.N:
        # while i < 11:
            s_col = []
            X_unvisited = self.get_random_samples(i, V_open)
            is_connected = False

            valid_nodes = []
            if i == 1:
                for y in X_unvisited:
                    new_node = self.Node(y)
                    if self.check_constraints(i, y, V_open[0]):
                        self.connect(i, V_open[0], new_node)
                        valid_nodes.append(new_node)
                    s_col.append(new_node)
                is_connected = True
            else:
                # if i < b:
                #     X_unvisited = X_unvisited[int(self.n_samples * a):]
                    # self.n_samples = n
                l_Xun = len(X_unvisited)
                for y in X_unvisited:
                    new_node = self.Node(y)
                    time3 = time.time()
                    near_parents = self.get_near_parents(i, y, V_open, l_Xun)

                    if i == self.N - 1:
                        near_parents = V_open
                    time4 = time.time()
                    parent_node, cost, k = self.find_parent(i, y, near_parents)

                    if parent_node is not None:
                        is_connected = True
                        new_node.cost = cost
                        time5 = time.time()
                        self.connect(i, parent_node, new_node)
                        valid_nodes.append(new_node)
                    s_col.append(new_node)

            sample_collection.append(s_col)

            if not is_connected:
                if tries < MAX_TRIES:
                    self.n_samples = int(self.n_samples * 1.1)
                    tries += 1
                else:
                    print('Reaching the max attempts!')
                    break
            else:
                V_open = valid_nodes
                i += 1

        # self.draw_static(sample_collection)

        if i == self.N:
            final_node = valid_nodes[0]
            self.optimal_solution(final_node)
            self._problem_data.return_code = ParameterizationReturnCode.Ok
            print('Find the optimal solution!')
        else:
            print('No solution found!')

    def get_random_samples(self, i, parent_nodes):
        if i == self.N - 1:
            return [self.xs_end.x]

        x_lower = self.K[i][0]
        x_upper = self.K[i][1]
        delta = x_upper - x_lower

        if delta < 0:
            return None
        else:
            # total = int(self.n_samples / self.ratio)
            total = 1000
            # total = self.n_samples
            random_nos = random.sample(range(1, total + 1), self.n_samples)
            # print(random_nos)
            # random_nos = random.sample(range(1, 200), self.n_samples)
            rnds = []
            for random_no in random_nos:
                rnds.append(x_lower + random_no / total * delta)
                # rnds.append(x_lower + random_no / 199 * delta)

            # sort the samples
            rnds.sort()

        return rnds

    def get_x_bound(self, i, prev2_u, prev_x):
        qs = self.qs[i - 1]
        qss = self.qss[i - 1]
        qsss = self.qsss[i - 1]
        delta = self.deltas[i - 1]
        x_bound_3 = self.order_3_constraint(qs, qss, qsss, delta, prev2_u, prev_x)
        x_bound_2 = self.order_2_constraint(qs, qss, delta, prev_x)
        x_min = np.max([x_bound_2[0], x_bound_3[0]])
        x_max = np.min([x_bound_2[1], x_bound_3[1]])

        return [x_min, x_max]

    def order_2_constraint(self, qs, qss, delta, prev_x):
        a = 0.5 * qs / delta
        b = (qss - a) * prev_x

        x_min = - 1000000
        x_max = 1000000
        constraint = self.constraints[0]
        for k in range(self.dof):
            if a[k] > 0:
                x_min = max(x_min, (constraint[k][0] - b[k]) / a[k])
                x_max = min(x_max, (constraint[k][1] - b[k]) / a[k])
            else:
                x_min = max(x_min, (constraint[k][1] - b[k]) / a[k])
                x_max = min(x_max, (constraint[k][0] - b[k]) / a[k])
        if x_min < 0:
            x_min = 0
        return [x_min, x_max]

    def order_3_constraint(self, qs, qss, qsss, delta, prev2_u, prev_x):
        a = sqrt(prev_x) * (qs + 3 * delta * qss) / (2 * delta ** 2)
        b = sqrt(prev_x) ** 3 * (qs / (2 * delta ** 2) + 3 * qss / (2 * delta) - qsss)
        c = qs * sqrt(prev_x) * prev2_u / delta
        constraint = self.constraints[1]

        x_min = - 1000000
        x_max = 1000000
        for k in range(self.dof):
            if a[k] > 0:
                x_min = max(x_min, (constraint[k][0] + b[k] + c[k]) / a[k])
                x_max = min(x_max, (constraint[k][1] + b[k] + c[k]) / a[k])
            else:
                x_min = max(x_min, (constraint[k][1] + b[k] + c[k]) / a[k])
                x_max = min(x_max, (constraint[k][0] + b[k] + c[k]) / a[k])
        if x_min < 0:
            x_min = 0
        return [x_min, x_max]

    def get_near_parents(self, i, y, V_open, l_Xun):
        # return V_open
        delta_y = (self.K[i][1] - self.K[i][0]) / l_Xun
        epson = 10
        lower = y - delta_y * epson
        upper = y + delta_y * epson
        lower_index = -1
        upper_index = -1
        n = len(V_open)
        for j in range(n):
            if V_open[j].x >= lower:
                lower_index = j
                break

        for j in range(n):
            if V_open[n - j - 1].x <= upper:
                upper_index = n - j - 1
                break
        if lower_index == -1:
            return None
        else:
            return V_open[lower_index: upper_index + 1]

    def find_parent(self, i, y, near_parents):
        if near_parents is None or len(near_parents) == 0:
            return None, None, None

        cost_list = []
        parents = []
        delta = self.deltas[i - 1]
        for node in near_parents:
            parents.append(node)
            cost = self.cost_accumulation(delta, node.x, y, node.cost)
            cost_list.append(cost)

        k = 0
        l = len(cost_list)
        while len(cost_list):
            min_cost = min(cost_list)
            min_index = cost_list.index(min_cost)
            if self.check_constraints(i, y, parents[min_index]):
                return parents[min_index], min_cost, k + 1
            else:
                k += 1
                parents.pop(min_index)
                cost_list.pop(min_index)
        return None, None, l

    def connect(self, i, parent_node, new_node):
        delta_s = self.deltas[i - 1]
        delta_xs = new_node.x - parent_node.x
        new_node.parent = parent_node
        sdd = 0.5 * delta_xs / delta_s
        if i == 1:
            sddd = sqrt(new_node.x) * sdd / delta_s
        else:
            sddd = sqrt(new_node.x) * (sdd - parent_node.sdd) / delta_s
        new_node.sdd = sdd
        new_node.sddd = sddd
        if i == 1:
            new_node.cost = self.cost_accumulation(delta_s, parent_node.x, new_node.x, parent_node.cost)

    def check_constraints(self, i, new_x, parent_node):
        delta_xs = new_x - parent_node.x
        # new_x sdd, actually is the parent node sdd
        sdd = 0.5 * delta_xs / self.deltas[i - 1]
        if i == 1:
            sddd = 0
        else:
            sddd = sqrt(parent_node.x) * (sdd - parent_node.sdd) / self.deltas[i - 2]

        # second order constraint
        constraint_2 = np.array(self.constraints[0])
        constraint_3 = np.array(self.constraints[1])
        qdd = self.qs[i - 1] * sdd + self.qss[i - 1] * parent_node.x
        if np.all(qdd >= constraint_2[:, 0]) and np.all(qdd <= constraint_2[:, 1]):
            qddd = self.qs[i - 1] * sddd + 3 * self.qss[i - 1] * sdd * sqrt(parent_node.x) \
                   + self.qsss[i - 1] * parent_node.x * sqrt(parent_node.x)
            if np.all(qddd >= constraint_3[:, 0]) and np.all(qddd <= constraint_3[:, 1]):
                return True

        return False

    def cost_accumulation(self, delta_s, parent_x, new_x, prev_cost):
        return 2 * delta_s / (sqrt(parent_x) + sqrt(new_x)) + prev_cost

    def optimal_solution(self, final_node):
        solution_reverse = [final_node]
        while final_node.parent is not None:
            solution_reverse.append(final_node.parent)
            final_node = final_node.parent
        solution = solution_reverse[::-1]
        xs_list = [node.x for node in solution]
        sdd_list = [node.sdd for node in solution]
        sddd_list = [node.sddd for node in solution]
        xs = np.array(xs_list)
        self.problem_data.sd_vec_j = np.sqrt(xs)
        self.problem_data.sdd_vec_j = np.array(sdd_list[1:])
        self.problem_data.sddd_vec_j = np.array(sddd_list[1:])

    def draw_static(self, samples):
        # plt.rcParams['savefig.dpi'] = 300
        # plt.rcParams['figure.dpi'] = 200
        plt.rcParams['figure.figsize'] = (10, 6.5)
        plt.rc('text', usetex=True)
        plt.rc('font', family='Times New Roman')
        samples.reverse()

        y = len(samples) - 1
        for sample_list in samples:
            parent = []
            for sample in sample_list:
                # plt.scatter(y, sample.x, s=1, c='red', zorder=3)
                if sample.parent is not None:
                    if sample.parent not in parent:
                        parent.append(sample.parent)
                    plt.plot([y - 1, y], [sample.parent.x, sample.x], '-g', zorder=1, linewidth=2)
                    plt.plot([y - 1, y], [self.K[y - 1][1], self.K[y][1]], 'grey', linestyle='--', zorder=2, linewidth=2)
            # print([node.x for node in parent])
            y -= 1
        font_size = 35
        # plt.xlabel('$s$ index', fontdict={'math_fontfamily': 'cm', "size": font_size}, fontname="Times New Roman",
        #            labelpad=15)
        # plt.ylabel('$x$', fontdict={'math_fontfamily': 'cm', "size": font_size}, labelpad=15, rotation=0)
        plt.xlabel('$s$ index', fontdict={"size": font_size}, labelpad=15)
        plt.ylabel('$x$', fontdict={"size": font_size}, labelpad=11, rotation=0)
        bwith = 1
        ax = plt.gca()
        ax.spines['bottom'].set_linewidth(bwith)
        ax.spines['left'].set_linewidth(bwith)
        ax.spines['top'].set_linewidth(bwith)
        ax.spines['right'].set_linewidth(bwith)
        ax.tick_params(which="major", length=10, width=1.0)

        plt.xticks(fontsize=font_size-3, usetex=True)
        plt.yticks(fontsize=font_size-3, usetex=True)
        plt.legend(["Edge", "$\mathcal{L}$"], ncol=1, prop={"family": "Times New Roman", "size": font_size},
                   bbox_to_anchor=(0.42, 0.5))
        plt.gcf().subplots_adjust(bottom=0.25)
        plt.gcf().subplots_adjust(left=0.15)
        plt.savefig(
            "H:/2-writing/stopp_v13/picture/traj_sample/stopp/random/e10" + time.strftime('%Y%m%d_%H-%M-%S') + ".pdf")
        plt.show()


if __name__ == '__main__':
    ps = np.loadtxt('D:\\research\projects\TOPP\jerk\sample_based\sample_based_v4\examples\ps.txt')
    pss = np.loadtxt('D:\\research\projects\TOPP\jerk\sample_based\sample_based_v4\examples\pss.txt')
    psss = np.loadtxt('D:\\research\projects\TOPP\jerk\sample_based\sample_based_v4\examples\psss.txt')
    xs_max = np.loadtxt('D:\\research\projects\TOPP\jerk\sample_based\sample_based_v4\examples\\xs.txt').reshape(-1, 1)
    ss = np.loadtxt('D:\\research\projects\TOPP\jerk\sample_based\sample_based_v4\examples\ss.txt')

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
    SamplePlanner = SamplingAlgorithm(ps, pss, psss, constraints_list, xs_range, ss)

    """ **********************************************************************************************
                                    test functions
    ****************************************************************************************************"""
    # rnds = SamplePlanner.get_random_samples(1)
    # print(rnds)
    final_node = SamplePlanner.planning()

    time1 = time.time()
    # SamplePlanner.optimal_solution(final_node)
    end_t = time.time()
    print('Consumption: ', end_t - start_t)

    # plot_graph.plot_c_space()
    # plot_graph.plot_s_space()

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
