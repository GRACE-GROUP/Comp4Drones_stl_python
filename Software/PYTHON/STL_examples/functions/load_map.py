import re

import numpy
import numpy as np


def load_map(filename, xy_res, z_res, margin):
    fid = open(filename, 'r')
    map = {}
    map['xy_res'] = xy_res
    map['z_res'] = z_res
    map['margin'] = margin
    map['m_grid'] = np.zeros((1, 4))

    m = margin
    map_dimension = []
    obstacles_from_file = np.zeros((100, 9))
    index = 0

    while True:
        line = fid.readline()
        if line == "":
            break

        if line.find('#') != -1:
            continue
        elif line.find('map_dimension') != -1:
            line = line.replace('map_dimension', '')
            temp = re.findall("(-?\d+(?:\.\d+)?)", line)
            B = []
            for i in temp:
                B.append(float(i))
            B = numpy.array(B)
            map_dimension = B
            x = np.arange(B[0], B[3] + 0.1, xy_res, dtype=float)
            y = np.arange(B[1], B[4] + 0.1, xy_res, dtype=float)
            z = np.arange(B[2], B[5] + 0.1, xy_res, dtype=float)

            map['xlen'] = len(x)
            map['ylen'] = len(y)
            map['zlen'] = len(z)
            map['dimension'] = map['xlen'] * map['ylen'] * map['zlen']

            m_grid = np.empty((map['xlen'], map['ylen'], map['ylen'], 4))
            m_grid.fill(np.NaN)

            for i in range(map['xlen']):
                for j in range(map['ylen']):
                    for k in range(map['zlen']):
                        m_grid[i, j, k, 0: 3] = np.concatenate(([x[i]], [y[j]], [z[k]]), axis=0)

        elif line.find('obstacles') != -1:
            line = line.replace('obstacles', '')
            temp = re.findall("(-?\d+(?:\.\d+)?)", line)

            temp2 = []
            for i in temp:
                temp2.append(float(i))
            temp2 = np.array(temp2)
            obstacles_from_file[index, :] = temp2
            index += 1
        else:
            continue

    fid.close()

    obstacles_from_file = obstacles_from_file[0:index, :]
    map['map_dimension'] = map_dimension
    map['obstacles'] = obstacles_from_file
    map['number_obstacles'] = index

    for p in range(index - 1):
        B = map_dimension
        b = obstacles_from_file[p, :]
        cb = 1000000 * b(7) + 1000 * b(8) + b(9)
        x_start = 1 + np.floor(((b[1] - B[1] - m) / xy_res))
        y_start = 1 + np.floor(((b[2] - B[2] - m) / xy_res))
        z_start = 1 + np.floor(((b[3] - B[3] - m) / z_res))

        x_end = 1 + np.floor(((b[4] - B[1] + m) / xy_res))
        y_end = 1 + np.floor(((b[5] - B[2] + m) / xy_res))
        z_end = 1 + np.floor(((b[6] - B[3] + m) / xy_res))

        for a in range(x_start, x_end):
            for b in range(y_start, y_end):
                for c in range(z_start, z_end):
                    if ((a > 0) and (b > 0) and (c > 0) and (a <= map['xlen']) and (b <= map['ylen']) and (
                            c <= map['zlen'])):
                        m_grid[a, b, c, 4] = cb

    map['m_grid'] = m_grid

    return map
