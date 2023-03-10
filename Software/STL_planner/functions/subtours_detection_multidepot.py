from typing import Dict

import numpy as np
from numpy import ndarray


def subtours_detection_multidepot(x: np.ndarray, edges: np.ndarray): # -> tuple[ndarray, dict, int]
    n_edges = edges.shape[0]
    edges_true = []
    for index, Flag in enumerate(x[0:n_edges] != 0):
        if Flag:
            edges_true.append(edges[index])
    for index, Flag in enumerate(x[0:n_edges] == 2):
        if Flag:
            edges_true.append(edges[index])

    edges_true = np.array(edges_true)
    n_edges_true = edges_true.shape[0]
    edges_tour: ndarray = np.zeros((n_edges_true, 3))
    node_current = 0
    n_subtour = 0

    for i in range(n_edges_true):
        tot = np.where(edges_true[:, 0: 2] == node_current)
        edge_current_id = tot[0]

        if len(edge_current_id) == 0:
            edge_current_id = [i for i, x in enumerate(edges_true > 0) if x.any()][0]
            n_subtour = n_subtour + 1
        else:
            edge_current_id = edge_current_id[0]
            if tot[1][0] == 1:
                # edge_current_id -= n_edges_true
                edges_true[edge_current_id, [0, 1]] = edges_true[edge_current_id, [1, 0]]

        node_current = edges_true[edge_current_id, 1]
        edges_tour[i, :] = edges_true[edge_current_id, :]
        edges_true[edge_current_id, :] = [-1, - 1, - 1]
    where = np.where(edges_tour[0:n_edges_true - 1, 1] != edges_tour[1:n_edges_true, 0])[0]
    if len(where):
        subtour_ini = [0, where[0] + 1, n_edges_true]
    else:
        subtour_ini = [0, n_edges_true]

    tours: dict[int, ndarray] = {}
    for i in range(n_subtour + 1):
        tours[i] = np.concatenate((
            edges_tour[subtour_ini[i]:subtour_ini[i + 1], 0], [edges_tour[subtour_ini[i], 0]]
        ), axis=0)

    print('=============================')
    print('Number of subtours: ', str(n_subtour))
    for i in range(n_subtour):
        print(['-Subtour ', i + 1, ' (nodes): ', str(tours[1 + i])])
    print('=============================')
    print(' ')
    print(' ')

    return edges_tour, tours, n_subtour
