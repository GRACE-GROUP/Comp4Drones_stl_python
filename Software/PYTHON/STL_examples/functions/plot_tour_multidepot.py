import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
import mpl_toolkits

from mpl_toolkits.mplot3d import Axes3D

def plot_tour_multidepot(initial_position: np.ndarray, goal_matrix: np.ndarray, n_drones: int,
                         edges_tour: np.ndarray) -> None:
    fig: Figure = plt.figure()
    axes: Axes = fig.add_axes((0.1, 0.1, 0.8, 0.8), projection='3d')
    for i in range(n_drones):
        # initial_position(1, i),initial_position(2, i) , initial_position(3, i), 'ro', 'linewidth', 4, 'markersize', 4
        #START-VM
        #mpl_toolkits.mplot3d.art3d.Patch3D(initial_position[0, i], initial_position[1, i], initial_position[2, i], 'ro', linewidth=4,
        #            markersize=4)
        axes.scatter(initial_position[0, i], initial_position[1, i], initial_position[2, i], c='red', marker='*', s=1000)
        #END-VM
    for i in range(goal_matrix.shape[0]):
        # goal_matrix(1:end,1),goal_matrix(1:end,2),goal_matrix(1:end,3),'ko','linewidth',4,'markersize',4
        axes.plot3D(goal_matrix[:, 0], goal_matrix[:, 1], goal_matrix[:, 2], 'ko', linewidth=4, markersize=4)

    for i in range(initial_position.shape[1]):
        # initial_position(1, i), initial_position(2, i), initial_position(3, i), ['  0 (Drone ' num2str(i) ')'],
        #      'fontsize', 12, 'fontweight', 'bold'
        axes.text(initial_position[0, i], initial_position[1, i], initial_position[2, i], '  0 (Drone ' + str(i) + ')',
                  fontsize=12, fontweight='bold')

    for i in range(goal_matrix.shape[0]):
        # goal_matrix(i,1) , goal_matrix(i,2) , goal_matrix(i,3) , ['  ' num2str(i)] , 'fontsize' , 12 , 'fontweight','bold'
        axes.text(goal_matrix[i, 0], goal_matrix[i, 1], goal_matrix[i, 2], '  ' + str(i),
                  fontsize=12, fontweight='bold')

    color_drone = np.array([[0.93, 0.69, 0.13],
                            [0.49, 0.18, 0.56],
                            [0.26, 0.72, 0.54],
                            [0.46, 0.32, 0.74]])

    for i in range(n_drones):
        nodes = np.concatenate(([initial_position[:, i].T], goal_matrix), axis=0)
        edges_tour_drone: np.ndarray = edges_tour[edges_tour[:, 2] == i + 1, :]
        for j in range(edges_tour_drone.shape[0]):
            P1: np.ndarray = nodes[int(edges_tour_drone[j, 0]), :]
            P2: np.ndarray = nodes[int(edges_tour_drone[j, 1]), :]
            D = P2 - P1
            # quiver3(P1(1), P1(2), P1(3), D(1), D(2), D(3), 0, 'MaxHeadSize', 5 / vecnorm(D, 2, 2), 'color',
            #         color_drone(i,:), 'linestyle', '-', 'linewidth', 2)
            axes.quiver(P1[0], P1[1], P1[2], D[0], D[1], D[2], color=color_drone[i, :], linestyle='-', linewidth=2,
                        arrow_length_ratio=.1)

    nodes = np.concatenate((initial_position.T, goal_matrix), axis=0)

    axes.set_aspect('auto', adjustable='box')
    axes.set_xlim3d(min(nodes[:, 0]) - 3, max(nodes[:, 0]) + 3)
    axes.set_ylim3d(min(nodes[:, 1]) - 3, max(nodes[:, 1]) + 3)
    axes.set_zlim3d(min(nodes[:, 2]) - 3, max(nodes[:, 2]) + 3)

    axes.set_xlabel('x [m]', fontsize=12, fontweight='bold')
    axes.set_ylabel('y [m]', fontsize=12, fontweight='bold')
    axes.set_zlabel('z [m]', fontsize=12, fontweight='bold')

    axes.grid(visible=True, which='minor')

    font = {'family': 'normal',
            'weight': 'bold',
            'size': 12}

    plt.rc('font', **font)

    axes.view_init(70, 40)
    fig.show()

    file_name = 'tour_x_y_z.png'
    result_dir = 'figures/'

    if not os.path.exists(result_dir):
        os.makedirs(result_dir)
    fig.savefig(result_dir + file_name)
