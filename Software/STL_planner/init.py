import math
from math import radians

import numpy as np
from numpy import ndarray

from functions.get_number_waypoints_multidepot import get_number_waypoints_multidepot
from functions.get_sequence_multidepot import get_sequence_multidepot
from functions.get_sequence_multidepot_cvx import get_sequence_multidepot_cvx
from functions.rot_z import rot_z
from missions.stl_reach_avoid import stl_reach_avoid

# print(' ')
# print('The init file allows to simulate a scenarios where four drones have to')
# print('inspect a power line mock-up while avoiding obstacles placed along the way')
# print('  1) Reach and avoid scenario (1 obstacle) considering 1 drone')
# print('  2) Reach and avoid scenario (1 obstacle) considering 2 drone')
# print('  3) Reach and avoid scenario (1 obstacle) considering 3 drone')
# print('---------------------------------------------------------------------------------')
#
# print(' ')
# selection = float(input('Which scenario do you intend to simulate?\n'))
# print(' ')
#
# enable_plot = input('Do you want to save the plots?y/n\n')
# print(' ')
#
# enable_saving_txt = input('Do you want to save the obtained waypoints and trajectories?y/n\n')
# print(' ')
#
# enable_traj_rot = input('Do you want to rotate and translate the obtained trajectories?y/n\n')
# print(' ')
#
# enable_animation = input('Do you want to run the animation?y/n\n')
# print(' ')
#
# enable_Belta_variation = input('Do you want to use Belta''s min/max approximations?y/n\n')
# print(' ')
#
# print(' ')
# print('Which estimation method for the minimum feasible time do you want to use?\n')
# print('---------------------------------------------------------------------------------')
# print(' 1 = reaching the targets with maximum velocity.  ')
# print(' 2 = reaching the targets with zero velocity (using splines). ')
# time_estimation_method = float(input(' \n'))
# print(' ')
#
# print(' ')
# print('Which solver do you want to use?\n')
# print('---------------------------------------------------------------------------------')
# print(' 1 = mumps.  ')
# print(' 2 = ma27. ')
# print(' 3 = ma57. ')
# solver_choice = float(input(' \n'))
# print(' ')
#
# enable_boolean = input('Do you want to run the optimization in boolean mode?y/n\n')
# print(' ')

selection = 1
enable_plot = 'y'
enable_saving_txt = 'y'
enable_traj_rot = 'y'
enable_animation = 'y'
enable_Belta_variation = 'y'
time_estimation_method = 1
solver_choice = 1
enable_boolean = 'y'

if selection == 1:
    # print('This scenario encodes the following STL formula:')
    # print(' ')
    # print('\varphi_\mathrm{sep}^{i,j} = \square_{[0, T]} \left(\lVert {^W\mathbf{r}}^i - {^W\mathbf{r}}^j')
    # print('      \rVert  \geq \delta_\mathrm{min} \right), ')
    # print('\varphi_\mathrm{mra} = \wedge_{k=1}^N \varphi_\mathrm{ra}^k \bigwedge \wedge_{k=1}^N \left(')
    # print('      \wedge_{i \neq j} \varphi_\mathrm{sep}^{i,j} \right)')
    # print(' ')

    drones: int = 1
    map_name: str = 'test_map_reach_avoid.txt'
    plotting_time: float = 0.05
    scale: float = 0.25
    destinationTxT: str = 'txt_files'

    angle_deg: int = 15
    angle_rad: float = radians(angle_deg)

    translation_x: int = -25
    translation_y: int = -25

    initial_position: ndarray = np.zeros((3, drones))
    for i in range(drones):
        initial_position[:, i] = [-1.25, -1.25, 1.75]

    initial_position_gazebo = np.zeros((initial_position.size, drones))

    rotation_gazebo = rot_z(angle_deg)

    for i in range(drones):
        a = np.array(initial_position[:, i])[np.newaxis]
        initial_position_gazebo[:, i] = (
            np.add(np.dot(rotation_gazebo, a.transpose()),
                   np.array([[translation_x], [translation_y], [0]]))).transpose()

    initial_velocity = np.zeros((3, drones))
    motion_time = 1
    sampling_time = 0.05

    delta_min = 0.1
    constraint_max_axis_vel = np.zeros((3, drones))
    constraint_max_axis_acc = np.zeros((3, drones))

    for i in range(drones):
        constraint_max_axis_vel[:, i] = [1 / 2.25, 1 / 2.25, 1 / 2.25]
        constraint_max_axis_acc[:, i] = np.ones((1, 3))

    max_vel = 7
    max_acc = 2
    C = 10
    C_dist = 5
    i = 0
    goal = {}

    goal[i] = {'stop': [1.75, 1.75, 0.75]}
    goal[i]['heading_angle'] = math.pi / 2
    goal[i]['ds'] = 0.15

    i += 1
    goal[i] = {'stop': [-1.75, 1.75, 0.75]}
    goal[i]['heading_angle'] = math.pi
    goal[i]['ds'] = 0.15

    i += 1
    goal[i] = {'stop': [1.75, -1.75, 0.75]}
    goal[i]['heading_angle'] = 3 * math.pi / 2
    goal[i]['ds'] = 0.15

    i += 1

    goal[i] = {'stop': [-1.75, -1.75, 0.75]}
    goal[i]['heading_angle'] = 0
    goal[i]['ds'] = 0.15

    number_targets: int = len(goal)

    for i in range(number_targets):
        goal[i]['lb'] = np.add(goal[i]['stop'], -1 * goal[i]['ds'])
        goal[i]['ub'] = np.add(goal[i]['stop'], goal[i]['ds'])

    sigma_max: int = 15

    # print(' ')
    # selection_CVX = input('Do you want to use CVX to compute the optimal sequence?y/n\n')
    # print(' ')
    selection_CVX = 'n'

    if selection_CVX == 'n':
        sequence: dict[int, np.ndarray] = get_sequence_multidepot(initial_position=initial_position, goal=goal,
                                                                  n_drones=drones, n_targets=number_targets,
                                                                  delta_max=sigma_max)

    elif selection_CVX == 'y':
        sequence = get_sequence_multidepot_cvx(initial_position, goal, drones, number_targets, sigma_max)
    else:
        sequence = get_sequence_multidepot_cvx(initial_position, goal, drones, number_targets, sigma_max)

    inspection_time = 5

    if time_estimation_method == 1:
        margin_time = 1.8
    elif time_estimation_method == 2:
        margin_time = 1.0
    else:
        margin_time = 1.0  # TODO:bayad break kone

    (WPs_sequence, WPs_total) = get_number_waypoints_multidepot(initial_position, goal, drones, sequence, max_vel,
                                                                max_acc, constraint_max_axis_vel,
                                                                constraint_max_axis_acc, margin_time, inspection_time,
                                                                motion_time, time_estimation_method)

    for k in range(drones):
        n_regions = len(sequence[k]) - 1
        I_installation = {}
        WPs_cumsum = np.cumsum(WPs_sequence[k])

        for i in range(n_regions):
            I_installation[i] = asd = np.arange(1 + round(WPs_cumsum[2 * i] / sampling_time),
                                                2 + round(WPs_cumsum[2 * i + 1] / sampling_time))

        # eval(['parameters.I_installation.Drone_' num2str(k) '= I_installation;'])

    print(' ')
    print('The goal regions are represented in blue, the obstacles in red, the drone starting');
    print('points are in magenta')
    print(' ')

    # stl_reach_avoid.py
elif selection == 2:
    # print('This scenario encodes the following STL formula:')
    # print(' ')
    # print('\varphi_\mathrm{sep}^{i,j} = \square_{[0, T]} \left(\lVert {^W\mathbf{r}}^i - {^W\mathbf{r}}^j')
    # print('      \rVert  \geq \delta_\mathrm{min} \right), ')
    # print('\varphi_\mathrm{mra} = \wedge_{k=1}^N \varphi_\mathrm{ra}^k \bigwedge \wedge_{k=1}^N \left(')
    # print('      \wedge_{i \neq j} \varphi_\mathrm{sep}^{i,j} \right)')
    # print(' ')

    drones = 2
    map_name = 'test_map_reach_avoid.txt'
    plotting_time = 0.05
    scale = 0.25
    destinationTxT = 'txt_files'

    angle_deg = 15
    angle_rad = radians(angle_deg)

    translation_x = -25
    translation_y = -25

    initial_position = np.zeros((3, drones))
    for i in range(drones):
        initial_position[:, i] = [-1.25, -1.25, 1.75]

    initial_position_gazebo = np.zeros((initial_position.size, drones))

    rotation_gazebo = rot_z(angle_deg)

    for i in range(drones):
        a = np.array(initial_position[:, i])[np.newaxis]
        initial_position_gazebo[:, i] = (
            np.add(np.dot(rotation_gazebo, a.transpose()),
                   np.array([[translation_x], [translation_y], [0]]))).transpose()

    initial_velocity = np.zeros((3, drones))
    motion_time = 1
    sampling_time = 0.05

    delta_min = 0.1
    constraint_max_axis_vel = np.zeros((3, drones))
    constraint_max_axis_acc = np.zeros((3, drones))

    for i in range(drones):
        constraint_max_axis_vel[:, i] = [1 / 2.25, 1 / 2.25, 1 / 2.25]
        constraint_max_axis_acc[:, i] = np.ones((1, 3))

    max_vel = 7
    max_acc = 2
    C = 10
    C_dist = 5
    i = 0
    goal = {}

    goal[i] = {'stop': [1.75, 1.75, 0.75]}
    goal[i]['heading_angle'] = math.pi / 2
    goal[i]['ds'] = 0.15

    i += 1
    goal[i] = {'stop': [1.75, 1.75, 0.75]}
    goal[i]['heading_angle'] = math.pi
    goal[i]['ds'] = 0.15

    i += 1
    goal[i] = {'stop': [1.75, -1.75, 0.75]}
    goal[i]['heading_angle'] = 3 * math.pi / 2
    goal[i]['ds'] = 0.15

    i += 1

    goal[i] = {'stop': [-1.75, -1.75, 0.75]}
    goal[i]['heading_angle'] = 0
    goal[i]['ds'] = 0.15

    number_targets = len(goal)

    for i in range(number_targets):
        goal[i]['lb'] = np.add(goal[i]['stop'], -1 * goal[i]['ds'])
        goal[i]['ub'] = np.add(goal[i]['stop'], goal[i]['ds'])

    sigma_max = 15

    # print(' ')
    # selection_CVX = input('Do you want to use CVX to compute the optimal sequence?y/n\n')
    # print(' ')
    selection_CVX = 'n'

    if selection_CVX == 'n':
        sequence = get_sequence_multidepot(initial_position=initial_position, goal=goal, n_drones=drones,
                                           n_targets=number_targets, delta_max=sigma_max)

    elif selection_CVX == 'y':
        sequence = get_sequence_multidepot_cvx(initial_position, goal, drones, number_targets, sigma_max)
    else:
        sequence = get_sequence_multidepot_cvx(initial_position, goal, drones, number_targets, sigma_max)

    inspection_time = 5

    if time_estimation_method == 1:
        margin_time = 1.8
    elif time_estimation_method == 2:
        margin_time = 1.0
    else:
        margin_time = 1.0  # TODO:bayad break kone

    (WPs_sequence, WPs_total) = get_number_waypoints_multidepot(initial_position, goal, drones, sequence, max_vel,
                                                                max_acc, constraint_max_axis_vel,
                                                                constraint_max_axis_acc, margin_time, inspection_time,
                                                                motion_time, time_estimation_method)

    for k in range(drones):
        n_regions = len(sequence[k]) - 1
        I_installation = {}
        WPs_cumsum = np.cumsum(WPs_sequence[k])

        for i in range(n_regions):
            I_installation[i] = asd = np.arange(1 + round(WPs_cumsum[2 * i] / sampling_time),
                                                2 + round(WPs_cumsum[2 * i + 1] / sampling_time))

        # eval(['parameters.I_installation.Drone_' num2str(k) '= I_installation;'])

    print(' ')
    print('The goal regions are represented in blue, the obstacles in red, the drone starting');
    print('points are in magenta')
    print(' ')

    # stl_reach_avoid.py
elif selection == 3:
    # print('This scenario encodes the following STL formula:')
    # print(' ')
    # print('\varphi_\mathrm{sep}^{i,j} = \square_{[0, T]} \left(\lVert {^W\mathbf{r}}^i - {^W\mathbf{r}}^j')
    # print('      \rVert  \geq \delta_\mathrm{min} \right), ')
    # print('\varphi_\mathrm{mra} = \wedge_{k=1}^N \varphi_\mathrm{ra}^k \bigwedge \wedge_{k=1}^N \left(')
    # print('      \wedge_{i \neq j} \varphi_\mathrm{sep}^{i,j} \right)')
    # print(' ')

    drones = 3
    map_name = 'test_map_reach_avoid.txt'
    plotting_time = 0.05
    scale = 0.25
    destinationTxT = 'txt_files'

    angle_deg = 15
    angle_rad = radians(angle_deg)

    translation_x = -25
    translation_y = -25

    initial_position = np.zeros((3, drones))
    for i in range(drones):
        initial_position[:, i] = [-1.25, -1.25, 1.75]

    initial_position_gazebo = np.zeros((initial_position.size, drones))

    rotation_gazebo = rot_z(angle_deg)

    for i in range(drones):
        a = np.array(initial_position[:, i])[np.newaxis]
        initial_position_gazebo[:, i] = (
            np.add(np.dot(rotation_gazebo, a.transpose()),
                   np.array([[translation_x], [translation_y], [0]]))).transpose()

    initial_velocity = np.zeros((3, drones))
    motion_time = 1
    sampling_time = 0.05

    delta_min = 0.1
    constraint_max_axis_vel = np.zeros((3, drones))
    constraint_max_axis_acc = np.zeros((3, drones))

    for i in range(drones):
        constraint_max_axis_vel[:, i] = [1 / 2.25, 1 / 2.25, 1 / 2.25]
        constraint_max_axis_acc[:, i] = np.ones((1, 3))

    max_vel = 7
    max_acc = 2
    C = 10
    C_dist = 5
    i = 0
    goal = {}

    goal[i] = {'stop': [1.75, 1.75, 0.75]}
    goal[i]['heading_angle'] = math.pi / 2
    goal[i]['ds'] = 0.15

    i += 1
    goal[i] = {'stop': [1.75, 1.75, 0.75]}
    goal[i]['heading_angle'] = math.pi
    goal[i]['ds'] = 0.15

    i += 1
    goal[i] = {'stop': [1.75, -1.75, 0.75]}
    goal[i]['heading_angle'] = 3 * math.pi / 2
    goal[i]['ds'] = 0.15

    i += 1

    goal[i] = {'stop': [-1.75, -1.75, 0.75]}
    goal[i]['heading_angle'] = 0
    goal[i]['ds'] = 0.15

    number_targets = len(goal)

    for i in range(number_targets):
        goal[i]['lb'] = np.add(goal[i]['stop'], -1 * goal[i]['ds'])
        goal[i]['ub'] = np.add(goal[i]['stop'], goal[i]['ds'])

    sigma_max = 15

    # print(' ')
    # selection_CVX = input('Do you want to use CVX to compute the optimal sequence?y/n\n')
    # print(' ')
    selection_CVX = 'n'

    if selection_CVX == 'n':
        sequence = get_sequence_multidepot(initial_position=initial_position, goal=goal, n_drones=drones,
                                           n_targets=number_targets, delta_max=sigma_max)

    elif selection_CVX == 'y':
        sequence = get_sequence_multidepot_cvx(initial_position, goal, drones, number_targets, sigma_max)
    else:
        sequence = get_sequence_multidepot_cvx(initial_position, goal, drones, number_targets, sigma_max)

    inspection_time = 5

    if time_estimation_method == 1:
        margin_time = 1.8
    elif time_estimation_method == 2:
        margin_time = 1.0
    else:
        margin_time = 1.0  # TODO:bayad break kone

    (WPs_sequence, WPs_total) = get_number_waypoints_multidepot(initial_position, goal, drones, sequence, max_vel,
                                                                max_acc, constraint_max_axis_vel,
                                                                constraint_max_axis_acc, margin_time, inspection_time,
                                                                motion_time, time_estimation_method)

    for k in range(drones):
        n_regions = len(sequence[k]) - 1
        I_installation = {}
        WPs_cumsum = np.cumsum(WPs_sequence[k])

        for i in range(n_regions):
            I_installation[i] = asd = np.arange(1 + round(WPs_cumsum[2 * i] / sampling_time),
                                                2 + round(WPs_cumsum[2 * i + 1] / sampling_time))

        # eval(['parameters.I_installation.Drone_' num2str(k) '= I_installation;'])

    print(' ')
    print('The goal regions are represented in blue, the obstacles in red, the drone starting');
    print('points are in magenta')
    print(' ')

#START-VM
#stl_reach_avoid('maps/' + map_name, selection, goal, drones, initial_position, motion_time, sampling_time, WPs_total,
#                max_vel, max_acc, constraint_max_axis_vel, constraint_max_axis_acc, delta_min, C, C_dist,
#                enable_Belta_variation, sequence, WPs_sequence,initial_velocity)

stl_reach_avoid('/Users/valeriomariani/Library/CloudStorage/Dropbox/Lavoro/Progetti/Ongoing/Comp4Drones/codeport/PYTHON/Software/PYTHON/STL_examples/maps/' + map_name, selection, goal, drones, initial_position, motion_time, sampling_time, WPs_total,
                max_vel, max_acc, constraint_max_axis_vel, constraint_max_axis_acc, delta_min, C, C_dist,
                enable_Belta_variation, sequence, WPs_sequence,initial_velocity)
#END-VM