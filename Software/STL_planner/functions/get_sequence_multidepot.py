import numpy as np

from functions.nchoosek import nchoosek_vector_2
from functions.intlinprog import find_solutions, MipModel
from functions.plot_tour_multidepot import plot_tour_multidepot
from functions.subtours_detection_multidepot import subtours_detection_multidepot


def get_sequence_multidepot(initial_position: np.ndarray, goal: dict, n_drones: int, n_targets: int,
                            delta_max: int): #  -> dict[int, np.ndarray]:
    edges_drone = nchoosek_vector_2(0, n_targets)
    n_edges_drone = edges_drone.shape[0]

    n_edges = n_edges_drone * n_drones

    if n_drones > 1:
        combos_drones = nchoosek_vector_2(1, n_drones)
        n_combos_drones = combos_drones.shape[0]
    else:
        n_combos_drones = 0

    goal_matrix = np.zeros(shape=(n_targets, 3))
    for i in range(n_targets):
        goal_matrix[i, :] = goal[i]['stop']

    edges = np.zeros((n_edges, 3))
    Cost = np.zeros((n_edges, 1))

    for i in range(n_drones):
        b = np.array([initial_position[:, i].T])
        nodes = np.concatenate((b, goal_matrix), axis=0)

        edges[n_edges_drone * i: n_edges_drone * (i + 1), :] = np.concatenate((edges_drone, (i + 1) * np.ones(
            shape=(n_edges_drone, 1))), axis=1)

        for j in range(n_edges_drone * i, n_edges_drone * (i + 1)):
            Cost[j] = np.linalg.norm(nodes[int(edges[j, 1]), :] - nodes[int(edges[j, 0]), :])

    objective_fcn = np.concatenate((Cost.T, np.ones((1, n_combos_drones)), np.zeros((1, n_targets * n_drones))), axis=1)
    Aeq_targets = np.zeros((n_targets, n_edges + n_combos_drones + n_targets * n_drones))

    for i in range(n_targets):
        for j in range(n_edges):
            if i + 1 in edges[j, 0: 2]:
                Aeq_targets[i, j] = 1

    beq_targets = 2 * np.ones((n_targets, 1))

    Aeq_targets_drone = np.zeros((n_targets * n_drones, n_edges + n_combos_drones + n_targets * n_drones))

    for k in range(n_drones):
        for i in range(n_targets):
            for j in range(n_edges):
                if i + 1 in edges[j, 0: 2] and edges[j, 2] == k + 1:
                    Aeq_targets_drone[k * n_targets + i, j] = 1

            Aeq_targets_drone[k * n_targets + i, n_edges + n_combos_drones + k * n_targets + i] = -2

    beq_targets_drone = np.zeros((n_targets * n_drones, 1))

    Aeq_depot = np.zeros((n_drones, n_edges + n_combos_drones + n_targets * n_drones))

    for i in range(n_drones):
        for j in range(n_edges):
            if 0 in edges[j, 0: 2] and edges[j, 2] == i + 1:
                Aeq_depot[i, j] = 1

    beq_depot = 2 * np.ones((n_drones, 1))
    Aeq = np.concatenate((Aeq_targets, Aeq_targets_drone, Aeq_depot), axis=0)
    beq = np.concatenate((beq_targets, beq_targets_drone, beq_depot), axis=0)

    A = np.zeros((2 * n_combos_drones, n_edges + n_combos_drones + n_targets * n_drones))

    for i in range(n_combos_drones):
        # TODO: (AB) inja n_combos_drones sefr bode check nashode to baadi hatman check beshe
        A[2 * i, n_edges_drone * (combos_drones[i, 0] - 1): n_edges_drone * combos_drones[i, 0]] = \
            Cost[n_edges_drone * (combos_drones[i, 0] - 1): n_edges_drone * combos_drones[i, 0]]
        A[2 * i, n_edges_drone * (combos_drones[i, 1] - 1): n_edges_drone * combos_drones[i, 1]] = \
            -1 * Cost[n_edges_drone * (combos_drones[i, 1] - 1): n_edges_drone * combos_drones[i, 1]]

        A[2 * i, n_edges + i] = -1

        A[2 * i + 1, n_edges_drone * (combos_drones[i, 1] - 1): n_edges_drone * combos_drones[i, 1]] = \
            Cost[n_edges_drone * (combos_drones[i, 1] - 1): n_edges_drone * combos_drones[i, 1]]
        A[2 * i + 1, n_edges_drone * (combos_drones[i, 0] - 1): n_edges_drone * combos_drones[i, 0]] = \
            -1 * Cost[n_edges_drone * (combos_drones[i, 0] - 1): n_edges_drone * combos_drones[i, 0]]

        A[2 * i + 1, n_edges + i] = -1

    b = np.zeros((2 * n_combos_drones, 1))

    intcon = np.concatenate((range(1, n_edges + 1), range(n_edges + n_combos_drones + 1,
                                                          n_edges + n_combos_drones + n_targets * n_drones + 1)),
                            axis=0)

    lb = np.zeros((n_edges + n_combos_drones + n_targets * n_drones, 1))
    ub = np.concatenate(
        (np.ones((n_edges, 1)), delta_max * np.ones((n_combos_drones, 1)), np.ones((n_targets * n_drones, 1))), axis=0)
    ub_temp = edges[:, 0] == 0

    for index, Flag in enumerate(ub_temp):
        if Flag:
            ub[index] = 2

    bounds = tuple((i[0], j[0]) for i, j in zip(lb, ub))

    minimize = True
    mip_model = MipModel(objective_fcn, minimize, A, b, Aeq, beq, bounds=bounds, int_vars=[])

    x = find_solutions(mip_model)
    x = np.round(x.x)

    edges_tour, tours, n_subtour = subtours_detection_multidepot(x, edges)

    while n_subtour > 0:
        A_p = np.zeros((n_subtour, n_edges + n_combos_drones + n_targets * n_drones))
        b_p = np.zeros((n_subtour, 1))

        for i in range(n_subtour):
            edges_sub = nchoosek_vector_2(int(tours[1 + i][0]), int(tours[1 + i][-2]))
            n_edges_sub = edges_sub.shape[0]
            # print(edges_sub.shape[0])
            # n_subtour = 0
            for j in range(edges_sub.shape[0]):
                if edges_sub[j, 0] > edges_sub[j, 1]:
                    edges_sub[j, [0, 1]] = edges_sub[j, [1, 0]]

            for j in range(n_edges_sub):
                result = np.where((edges[:, [0, 1]] == edges_sub[j]).all(axis=1))[0]
                if len(result):
                    A_p[i, result[0]] = 1

            b_p[i] = tours[1 + i].shape[0] - 2

        A = np.concatenate((A, A_p), axis=0)
        b = np.concatenate((b, b_p), axis=0)

        mip_model = MipModel(objective_fcn, minimize, A, b, Aeq, beq, bounds=bounds, int_vars=[])
        #TODO: (VM) After debugging with Giuseppe we realized that the solution given here differs from that with linprog in matlab.
        x = find_solutions(mip_model)
        x = np.round(x.x)
        
        edges_tour, tours, n_subtour = subtours_detection_multidepot(x, edges)

    plot_tour_multidepot(initial_position, goal_matrix, n_drones, edges_tour)

    sequence_ini = np.where(tours[0] == 0)[0]
    sequence = {}
    for i in range(n_drones):
        sequence[i] = tours[0][sequence_ini[i]:sequence_ini[i + 1] + 1].T
    return sequence
