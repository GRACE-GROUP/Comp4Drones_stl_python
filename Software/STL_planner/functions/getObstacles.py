import re

import numpy as np


def getObstacles(map, varargin=None):
    if varargin is None:
        varargin = []
    ext_obstacles_flag = 0
    ext_obstacles: np.ndarray = np.array([])

    if len(varargin):
        ext_obstacles = varargin[1]
        ext_obstacles_flag = 1

    fid = open(map, 'r')

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
            continue
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
    obstacles_from_file = obstacles_from_file[0: 6]
    obstacles_from_file = np.array([obstacles_from_file])

    N = obstacles_from_file.shape[0]

    obstacles = {}
    A = np.concatenate((-np.eye(3), np.eye(3)), axis=0)

    for i in range(N):
        temp = {'lb': obstacles_from_file[i, 0:3], 'ub': obstacles_from_file[i, 3:6], 'A': A}
        temp['b'] = np.array([np.concatenate((-temp['lb'], temp['ub']), axis=0)]).transpose()
        # obstacles[i]['shape'] = Polyhedron('lb', obstacles[i]['lb'], 'ub', obstacles[i]['ub'])
        obstacles[i] = temp

    if ext_obstacles_flag and ext_obstacles.shape[0]:
        for i in range(N, ext_obstacles[:, 1].shape[0] + N):
            obstacles[i]['lb'] = obstacles_from_file[i - N, 1:3]
            obstacles[i]['ub'] = obstacles_from_file[i - N, 4:6]
            obstacles[i]['A'] = A
            obstacles[i]['b'] = np.concatenate((-obstacles[i]['lb'], obstacles[i]['ub']), axis=1).transpose()
            # obstacles[i]['shape'] = Polyhedron('lb', obstacles[i]['lb'], 'ub', obstacles[i]['ub'])

    return obstacles
