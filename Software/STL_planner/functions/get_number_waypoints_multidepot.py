import numpy as np


def get_number_waypoints_multidepot(initial_position: np.ndarray, goal: dict, n_drones: int,
                                    sequence: dict,
                                    max_vel: int, max_acc: int, constraint_max_axis_vel: np.ndarray,
                                    constraint_max_axis_acc: np.ndarray, margin_time: float, installation_time: int,
                                    motion_time: int, time_estimation_method: int) -> (dict, int):
    WPs_sequence = {}
    n_WPs_sequence = np.zeros((n_drones, 1))
    for k in range(n_drones):
        goal_vector = np.zeros((sequence[k].shape[0] - 2, 3))
        for i in range(1, sequence[k].shape[0] - 1):
            goal_vector[i - 1, :] = goal[sequence[k][i] - 1]['stop']

        positions = np.concatenate(([initial_position[:, k].T], goal_vector, [initial_position[:, k].T]), axis=0)
        difference_position = np.diff(positions, axis=0)

        vel_max_axis = max_vel * constraint_max_axis_vel[:, k]
        acc_max_axis = max_acc * constraint_max_axis_acc[:, k]

        Constant = (np.sqrt(3) - 1) / (2 * np.sqrt(3))

        t_min_sequence_navigation = np.zeros((1, difference_position.shape[0]))

        for i in range(t_min_sequence_navigation.shape[1]):
            t_min_axis = np.zeros((1, 3))
            for j in range(3):
                if time_estimation_method == 1:
                    t_min_1 = np.sqrt(2 * abs(difference_position[i, j]) / acc_max_axis[j])
                    t_min_2 = abs(difference_position[i, j]) / vel_max_axis[j] + vel_max_axis[j] / (2 * acc_max_axis[j])

                    if t_min_1 >= 0 and t_min_1 <= vel_max_axis[j] / acc_max_axis[j]:
                        t_min_axis[0][j] = t_min_1
                    elif t_min_2 > vel_max_axis[j] / acc_max_axis[j]:
                        t_min_axis[0][j] = t_min_2
                    else:
                        return
                elif time_estimation_method == 2:
                    t_min_vel = 15 / 8 * abs(difference_position[i, j]) / vel_max_axis[j]
                    t_min_acc = np.sqrt(60 * Constant * (2 * Constant ** 2 - 3 * Constant + 1) * abs(
                        difference_position[i, j]) / acc_max_axis[j])
                    t_min_axis[0][j] = max(t_min_vel, t_min_acc)
                else:
                    return
            t_min_sequence_navigation[0][i] = max(t_min_axis[0])

        t_min_sequence = np.zeros((1, 2 * t_min_sequence_navigation.shape[1]))

        for i in range(t_min_sequence_navigation.shape[1]):
            t_min_sequence[0][2 * i] = margin_time * t_min_sequence_navigation[0][i]
            t_min_sequence[0][2 * i + 1] = installation_time

        WPs_sequence[k] = np.ceil(t_min_sequence[0] / motion_time)
        n_WPs_sequence[k] = np.sum(WPs_sequence[k])

    WPs_total = np.max(n_WPs_sequence)
    for k in range(n_drones):
        diff_WPs = WPs_total - n_WPs_sequence[k]
        WPs_sequence[k][-1] = WPs_sequence[k][-1] + diff_WPs

    return WPs_sequence, WPs_total
