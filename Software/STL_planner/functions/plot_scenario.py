import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def plot_scenario(map, path):
    vertex_x = np.zeros((4, 6 * map['number_obstacles']))
    vertex_y = np.zeros((4, 6 * map['number_obstacles']))
    vertex_z = np.zeros((4, 6 * map['number_obstacles']))

    for i in range(map['number_obstacles']):
        lowerLimit = map['obstacles'][i, 0:3]
        upperLimit = map['obstacles'][i, 3:6]
        vertices = []

        vertices.append([lowerLimit[0], lowerLimit[1], lowerLimit[2]])
        vertices.append([upperLimit[0], lowerLimit[1], lowerLimit[2]])
        vertices.append([upperLimit[0], lowerLimit[1], upperLimit[2]])
        vertices.append([lowerLimit[0], lowerLimit[1], upperLimit[2]])
        vertices.append([lowerLimit[0], upperLimit[1], upperLimit[2]])
        vertices.append([upperLimit[0], upperLimit[1], upperLimit[2]])
        vertices.append([upperLimit[0], upperLimit[1], lowerLimit[2]])
        vertices.append([lowerLimit[0], upperLimit[1], lowerLimit[2]])

        vertices = np.array(vertices)

        start_y = (i) * 6

        vertex_x[:, start_y:start_y + 6] = [
            [vertices[0, 0], vertices[1, 0], vertices[2, 0], vertices[0, 0], vertices[0, 0], vertices[4, 0]],
            [vertices[1, 0], vertices[2, 0], vertices[5, 0], vertices[7, 0], vertices[1, 0], vertices[7, 0]],
            [vertices[2, 0], vertices[5, 0], vertices[4, 0], vertices[4, 0], vertices[6, 0], vertices[6, 0]],
            [vertices[3, 0], vertices[6, 0], vertices[3, 0], vertices[3, 0], vertices[7, 0], vertices[6, 0]]
        ]

        vertex_y[:, start_y:start_y + 6] = [
            [vertices[0, 1], vertices[1, 1], vertices[2, 1], vertices[0, 1], vertices[0, 1], vertices[4, 1]],
            [vertices[1, 1], vertices[2, 1], vertices[5, 1], vertices[7, 1], vertices[1, 1], vertices[7, 1]],
            [vertices[2, 1], vertices[5, 1], vertices[4, 1], vertices[4, 1], vertices[6, 1], vertices[6, 1]],
            [vertices[3, 1], vertices[6, 1], vertices[3, 1], vertices[3, 1], vertices[7, 1], vertices[6, 1]]
        ]
        vertex_z[:, start_y:start_y + 6] = [
            [vertices[0, 2], vertices[1, 2], vertices[2, 2], vertices[0, 2], vertices[0, 2], vertices[4, 2]],
            [vertices[1, 2], vertices[2, 2], vertices[5, 2], vertices[7, 2], vertices[1, 2], vertices[7, 2]],
            [vertices[2, 2], vertices[5, 2], vertices[4, 2], vertices[4, 2], vertices[6, 2], vertices[6, 2]],
            [vertices[3, 2], vertices[6, 2], vertices[3, 2], vertices[3, 2], vertices[7, 2], vertices[6, 2]]
        ]

    figure = plt.gcf()
    ax = Axes3D(figure, auto_add_to_figure=False)
    ax.set_xlabel('x', fontsize=12, fontweight='bold')
    ax.set_ylabel('y', fontsize=12, fontweight='bold')
    ax.set_zlabel('z', fontsize=12, fontweight='bold')

    figure.add_axes(ax)

    for i in range(len(path)):
        if len(path[i]):
            plt.plot(path[i][:, 0], path[i][:, 1], path[i][:, 2], '-', 'linewidth', 3)

    verts = [list(zip(vertex_x[1], vertex_y[1], vertex_z[1]))]

    data = map['map_dimension']
    ax.set_xlim(data[0], data[3])
    ax.set_ylim(data[1], data[4])
    ax.set_zlim(data[2], data[5])

    ax.add_collection3d(Poly3DCollection(verts))
    plt.grid()

    plt.show()
