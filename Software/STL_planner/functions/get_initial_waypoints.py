import cvxpy as cp
import numpy as np


def get_initial_waypoints(initial_state, optimizationParameters, selection, iteration):
    WPs_total = optimizationParameters['WPs_total']
    motion_time = optimizationParameters['motion_time']
    M = optimizationParameters['M']
    acc_bar = optimizationParameters['acc_bar']
    max_vel = optimizationParameters['max_vel']
    max_acc = optimizationParameters['max_acc']
    constraint_max_axis_vel = optimizationParameters['constraint_max_axis_vel']
    constraint_max_axis_acc = optimizationParameters['constraint_max_axis_acc']
    map = optimizationParameters['map']
    goal = optimizationParameters['goal']

    if selection == 1 or selection == 2 or selection == 3:
        sequence = optimizationParameters['sequence']
        WPs_sequence = optimizationParameters['WPs_sequence']

    dv = 0
    da = 0

    pp = cp.Variable(shape=(int(WPs_total + 1), 3), name='pp')
    vv = cp.Variable(shape=(int(WPs_total + 1), 3), name='vv')

    if selection == 1 or selection == 2 or selection == 3:
        objective = cp.Minimize(
            cp.sum(cp.abs(pp[1:-1, 0] - pp[0:-2, 0])) + cp.sum(cp.abs(pp[1:-1, 1] - pp[0:-2, 1])) + cp.sum(
                cp.abs(pp[1:-1, 2] - pp[0:-2, 2])))

    pp[0, :] == initial_state[0: 3]
    vv[0, :] == initial_state[3: 6]

    for i in range(1, int(WPs_total) + 1):
        for j in range(3):
            pp[i, j] >= map['map_dimension'][j]
            pp[i, j] <= map['map_dimension'][j + 3]

            alfa = M[0, :] * [[(pp[i, j] - pp[i - 1, j] - vv[i - 1, j] * motion_time)], [dv], [da]]
            beta = M[1, :] * [[(pp[i, j] - pp[i - 1, j] - vv[i - 1, j] * motion_time)], [dv], [da]]
            gamma = M[2, :] * [[(pp[i, j] - pp[i - 1, j] - vv[i - 1, j] * motion_time)], [dv], [da]]

            acc_bar * (pp[i, j] - pp[i - 1, j]) - motion_time * acc_bar * vv[i - 1, j] <= max_acc * \
            constraint_max_axis_acc[j, iteration]
            acc_bar * (pp[i, j] - pp[i - 1, j]) - motion_time * acc_bar * vv[i - 1, j] >= -max_acc * \
            constraint_max_axis_acc[j, iteration]

            vv[i, j] == (alfa / 24) * motion_time ** 4 + (beta / 6) * motion_time ** 3 + (
                    gamma / 2) * motion_time ** 2 + vv[i - 1, j]

            vv[i, j] <= max_vel * constraint_max_axis_vel[j, iteration]
            vv[i, j] >= -max_vel * constraint_max_axis_vel[j, iteration]
    if selection == 1 or selection == 2 or selection == 3:
        WPs_cumsum = np.cumsum(WPs_sequence[iteration])
        n_regions = len(sequence[iteration]) - 1
        for i in range(n_regions):
            for j in range(WPs_cumsum[2 * i], WPs_cumsum[2 * i + 1]):
                if sequence['iteration'][1 + i] == 0:
                    pp[j, :] == initial_state[0: 3].T
                else:
                    pp[j, :] == goal[sequence['iteration'][1 + i]].stop.T
    pp = np.array(pp)
    vv = np.array(vv)

    p0 = pp.reshape((WPs_total + 1) * 3, 1)
    v0 = vv.reshape((WPs_total + 1) * 3, 1)

    return p0, v0
