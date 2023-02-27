#START-VM
#import casadi as casadi
import casadi as ca
#END-VM

import pip

#START-VM
#import casadi
#END-VM

# Commented out IPython magic to ensure Python compatibility.
import numpy as np
import time
import math
#START-VM
import matplotlib
matplotlib.interactive(True)
#END-VM
from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import mpl_toolkits

from mpl_toolkits.mplot3d import Axes3D

from functions.getObstacles import getObstacles
from functions.get_initial_waypoints import get_initial_waypoints
from functions.load_map import load_map
from functions. plot_scenario import plot_scenario


def stl_reach_avoid(map_name, selection, goal, drones, initial_position, motion_time, sampling_time, WPs_total,
                    max_vel, max_acc, constraint_max_axis_vel, constraint_max_axis_acc, delta_min, C, C_dist,
                    enable_Belta_variation, sequence, WPs_sequence, initial_velocity):
    print(motion_time)
    print('Initialization...')

    obstacles = getObstacles(map_name)

    map = load_map(map_name, .5, .5, 0)

    NLP_results: Figure = plt.figure()
    axes: Axes = NLP_results.add_axes((0.1, 0.1, 0.8, 0.8), projection='3d')

    if selection == 1 or selection == 2 or selection == 3:
        plot_scenario(map, [])

    goal_elements = len(goal)
    obstacle_elements = len(obstacles)

    for i in range(goal_elements):
        # goal['i']['polyhedron'] = = Polyhedron('lb', obstacles[i]['lb'], 'ub', obstacles[i]['ub'])
        # plt.plot(goal['i']['polyhedron'], 'color', 'blue', 'alpha', 0.5)
        pass

    for i in range(obstacle_elements):
        # plt.plot(goal['i']['shape'], 'color', 'red', 'alpha', 0.5)
        pass
    initial_pos = {}
    for i in range(drones):
        # initial_pos['shape'] = Polyhedron('lb', initial_position[:, i] - goal[i]['ds'], 'ub',
        #                                   initial_position[:, i] + goal[i]['ds'])
        # plt.plot(initial_pos['shape'], 'color', 'magenta', 'alpha', 0.5)
        pass
    data = {'dimension': map['map_dimension']}
    # data['shape'] = Polyhedron('lb', data['dimension'][0:3], 'ub', data['dimension'][3:6])
    # plt.plot(data['shape'], 'EdgeColor', [0,0.5,0], 'alpha', 0, 'LineWidth', 2)
    marg = 0.25
    axes.axis([data['dimension'][0] - marg, data['dimension'][3] + marg, data['dimension'][1] - marg,
               data['dimension'][4] + marg])
    axes.set_zlim(data['dimension'][2] - marg, data['dimension'][5] + marg)
    axes.set_xlabel('x [m]', fontsize=12, fontweight='bold')
    axes.set_ylabel('y [m]', fontsize=12, fontweight='bold')
    axes.set_zlabel('z [m]', fontsize=12, fontweight='bold')
    axes.set_title('NLP solution')
    axes.view_init(70, 40)

    NLP_results.show()

    LP_results: Figure = plt.figure()
    axes: Axes = LP_results.add_axes((0.1, 0.1, 0.8, 0.8), projection='3d')

    if selection == 1 or selection == 2 or selection == 3:
        plot_scenario(map, [])

    goal_elements = len(goal)
    obstacle_elements = len(obstacles)

    for i in range(goal_elements):
        # goal['i']['polyhedron'] = = Polyhedron('lb', obstacles[i]['lb'], 'ub', obstacles[i]['ub'])
        # plt.plot(goal['i']['polyhedron'], 'color', 'blue', 'alpha', 0.5)
        pass

    for i in range(obstacle_elements):
        # plt.plot(goal['i']['shape'], 'color', 'red', 'alpha', 0.5)
        pass
    initial_pos = {}
    for i in range(drones):
        # initial_pos['shape'] = Polyhedron('lb', initial_position[:, i] - goal[i]['ds'], 'ub',
        #                                   initial_position[:, i] + goal[i]['ds'])
        # plt.plot(initial_pos['shape'], 'color', 'magenta', 'alpha', 0.5)
        pass
    data = {'dimension': map['map_dimension']}
    # data['shape'] = Polyhedron('lb', data['dimension'][0:3], 'ub', data['dimension'][3:6])
    # plt.plot(data['shape'], 'EdgeColor', [0,0.5,0], 'alpha', 0, 'LineWidth', 2)
    marg = 0.25
    axes.axis([data['dimension'][0] - marg, data['dimension'][3] + marg, data['dimension'][1] - marg,
               data['dimension'][4] + marg])
    axes.set_zlim(data['dimension'][2] - marg, data['dimension'][5] + marg)
    axes.set_xlabel('x [m]', fontsize=12, fontweight='bold')
    axes.set_ylabel('y [m]', fontsize=12, fontweight='bold')
    axes.set_zlabel('z [m]', fontsize=12, fontweight='bold')
    axes.set_title('LP solution')
    axes.view_init(80, 10)

    LP_results.show()
    M = 1 / (2 * motion_time ** 5) * np.array([[90, 0, - 15 * motion_time ** 2],
                                               [-90 * motion_time, 0, 15 * motion_time ** 3],
                                               [30 * motion_time ** 2, 0, - 3 * motion_time ** 4]])

    t_prime_a = (1 - 3 ** 0.5 / 3) * motion_time
    acc_bar = (90 / 12) * t_prime_a ** 3 / motion_time ** 5 - (90 / 4) * t_prime_a ** 2 / motion_time ** 4 + (
            30 / 2) * t_prime_a / motion_time ** 3

    da = 0
    dv = 0

    time = np.arange(sampling_time, motion_time + sampling_time, sampling_time)
    #START-VM
    WPs_total = int(WPs_total)
    #END-VM
    number_steps = WPs_total * time.size + 1
    
    Clen = 3 * (WPs_total + 1)
    

    optimizationParameters = {}
    optimizationParameters['sampling_time'] = sampling_time
    optimizationParameters['motion_time'] = motion_time
    optimizationParameters['M'] = M
    optimizationParameters['acc_bar'] = acc_bar
    optimizationParameters['max_vel'] = max_vel
    optimizationParameters['max_acc'] = max_acc
    optimizationParameters['da'] = da
    optimizationParameters['dv'] = dv
    optimizationParameters['obstacle_elements'] = obstacle_elements
    optimizationParameters['goal_elements'] = goal_elements
    optimizationParameters['WPs_total'] = WPs_total
    optimizationParameters['goal'] = goal
    #START-VM
    #WARNING!!! The following assignment is based on Giuseppe's explanation
    optimizationParameters['home'] = {key: {'goal_lb_N': np.array((number_steps,3)), 'goal_ub_N': np.array((number_steps,3))} for key in range(drones)}
    #END-VM
    optimizationParameters['obstacles'] = obstacles
    optimizationParameters['map'] = map
    optimizationParameters['constraint_max_axis_vel'] = constraint_max_axis_vel
    optimizationParameters['constraint_max_axis_acc'] = constraint_max_axis_acc
    optimizationParameters['initial_position'] = initial_position
    optimizationParameters['drones'] = drones
    optimizationParameters['delta_min'] = delta_min
    optimizationParameters['Clen'] = Clen
    optimizationParameters['C'] = C
    optimizationParameters['C_dist'] = C_dist
    optimizationParameters['enable_Belta_variation'] = enable_Belta_variation

    if selection == 1 or selection == 2 or selection == 3:
        optimizationParameters['sequence'] = sequence
        optimizationParameters['WPs_sequence'] = WPs_sequence

    print('Getting initial solution...')

    p0 = np.empty((0, 1))
    v0 = np.empty((0, 1))

    for i in range(drones):
        # temp_p, temp_v = get_initial_waypoints(np.concatenate((initial_position[:, i], initial_velocity[:, i]), axis=0),
        #                                        optimizationParameters, selection, i)
        temp_p = np.array([[0]])
        temp_v = np.array([[0]])

        if sum(temp_p) > 0 or sum(temp_v) > 0:
            print('Init unfeasible')
            return

        p0 = np.concatenate((p0, temp_p), axis=0)
        v0 = np.concatenate((v0, temp_v), axis=0)

    var0 = np.concatenate((p0, v0), axis=0)
    ############### %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    print('Trajectory generation...')
    optimizationParameters['obstacles_lb_N'] = [None]*obstacle_elements
    optimizationParameters['obstacles_ub_N'] = [None]*obstacle_elements
    
    for i in range(obstacle_elements):
        optimizationParameters['obstacles_lb_N'][i] = np.tile(obstacles[i]['lb'],(number_steps, 1))
        optimizationParameters['obstacles_ub_N'][i] = np.tile(obstacles[i]['ub'],(number_steps, 1))
    # Filling lower and upper bounds for the goal (target) regions
    for i in range(goal_elements): 
        optimizationParameters['goal'][i]['goal_lb_N'] = np.tile(goal[i]['lb'], (number_steps, 1))
        optimizationParameters['goal'][i]['goal_ub_N'] = np.tile(goal[i]['ub'], (number_steps, 1))

    # Filling lower and upper bounds for the home (target) regions
    #START-VM
    home = {}
    #END-VM
    for i in range(drones):
        #START-VM
        #home['lb'] = initial_position[:,i] - goal['ds']
        home['lb'] = initial_position[:,i] - goal[0]['ds']
        #END-VM
        home['ub'] = initial_position[:,i] + goal[0]['ds']
        
        optimizationParameters['home'][i]['goal_lb_N'] = np.tile(home['lb'], (number_steps, 1))
        optimizationParameters['home'][i]['goal_ub_N'] = np.tile(home['ub'], (number_steps, 1))


    # Variable initialization
    p   = []  # initialization position vector 
    v   = []  # initialization velocity vector
    lbp = []  # initialization lower bound position
    ubp = []  # initialization upper bound position
    lbv = []  # initialization lower bound velocity
    ubv = []  # initialization upper bound velocity
    g   = []  # initialization g function
    lbg = []  # initialization lower bound g function
    ubg = []  # initialization upper bound g function

    # This loop runs over all the available drones
    for j in range(drones):
        
        # The symbolic variable p_j_0, with j goes from 1 to the number of
        # available drones
        # Symbolic expression of the problem. More information about the MX.sym and
        # MX.zeros commands are available here: https://web.casadi.org/docs/
        
        #START-VM
        #initial_position_symbolic = ca.MX.sym(['p_' ,str(j), '_' ,str(0)], 3, 1);
        initial_position_symbolic = ca.MX.sym('p_'+str(j)+'_'+str(0), 3, 1);
        #END-VM
        
        p.append(initial_position_symbolic)  # p is initialized as empty vector
        
        lbp.append(initial_position[:,j])
        ubp.append(initial_position[:,j]) #VM TODO: upperbounds should be loaded from the txt file in /maps
        
        # The symbolic variable v_j_0, with j goes from 1 to the number of
        # available drones
        #START-VM
        #initial_velocity_symbolic = ca.MX.sym(['v_' ,str(j) ,'_', str(0)], 3, 1);
        initial_velocity_symbolic = ca.MX.sym('v_'+str(j)+'_'+str(0), 3, 1);
        #END-VM

        v.append(initial_velocity_symbolic)  # v is initialized as empty vector
        
        lbv.append(initial_velocity[:,j])
        ubv.append(initial_velocity[:,j])
        
        # This loop runs over all the available waypoints. The trajectory per
        # each drone is dived in N topic waypoints
        for k in range(WPs_total):
            
            # The symbolic variabile p_j_k, with j goes from 1 to the number of
            # available drones and k goes from 1 to the M wayapoints
            #START-VM
            #position_symbolic = ca.MX.sym(['p_' ,str(j), '_' ,str(k)], 3, 1);
            position_symbolic = ca.MX.sym('p_'+str(j)+'_'+str(k+1), 3, 1);
            #END-VM
            p.append(position_symbolic)
            
            # Overall bounds on movement. In particular, bounds on the position that cannot
            # exceed the dimension of the map
            lbp.append(np.transpose(map['map_dimension'][0:3])) # lower bounds on the map along each axis
            ubp.append(np.transpose(map['map_dimension'][3:6])) # upper bounds on the map along each axis
            
            # The symbolic variabile v_j_k, with j goes from 1 to the number of
            # available drones and k goes from 1 to the M wayapoints
            #START-VM
            #velocity_symbolic = ca.MX.sym(['v_', str(j) ,'_' ,str(k)], 3, 1)
            velocity_symbolic = ca.MX.sym('v_'+str(j)+'_'+str(k), 3, 1)
            #END-VM
            v.append(velocity_symbolic)
            
            # There are no bounds on the acceleration because there are no
            # symbolic variables accounting for it in the problem formulation
            # This allows accounting for the bounds on the velocity at the WPs
            lbv.append(-max_vel*constraint_max_axis_vel[:,j]) # lower bounds on the velocity
            ubv.append(max_vel*constraint_max_axis_vel[:,j]) # upper bounds on the velocity
            
            # distance variation for all axes, i.e., p = p{k} - p{k-1}. This
            # is possible because the problem is decoupled along each axis of
            # the inertial frame.
            # The variable Clen allows to shift the vector depending on the
            # number of drones (j accounts for it in the first loop). The
            # numbers 1, 2, and 3 refer to the x, y, and z-axis, respectively.
            
            #START-VM TODO: possible issues: p is not well shaped and
            #in matlab arrays start from 1 and not 0 as in python
            #k * 3 + 1 + (j-1) * Clen = 4, k=1, j-1=0, Clen=129
            #difference_p_x = p[k * 3 + 1 + (j-1) * Clen] - p[(k-1) * 3 + 1 + (j-1) * Clen] # along x-axis
            #difference_p_y = p[k * 3 + 2 + (j-1) * Clen] - p[(k-1) * 3 + 2 + (j-1) * Clen] # along y-axis
            #difference_p_z = p[k * 3 + 3 + (j-1) * Clen] - p[(k-1) * 3 + 3 + (j-1) * Clen] # along z-axis
            #above the original code: let's fix first the issue with p
            difference_p_x = p[k+1][0] - p[k][0] # along x-axis
            difference_p_y = p[k+1][1] - p[k][1] # along y-axis
            difference_p_z = p[k+1][2] - p[k][2] # along z-axis
            #END-VM
            
            # velocity variation for all axes, i.e.,v{k} - v{k-1}. This
            # is possible because the problem is decoupled along each axis of
            # the inertial frame. 
            # As before, the variable Clen allows shifting the vector depending
            # on the number of drones (the first loop runs over it)

            #START-VM
            #v_x_k = v[k * 3 + 1 + (j-1) * Clen]  # v{k} along x-axis
            #v_y_k = v[k * 3 + 2 + (j-1) * Clen]  # v{k} along y-axis
            #v_z_k = v[k * 3 + 3 + (j-1) * Clen]  # v{k} along z-axis
            #
            ## km1 stands for k minus 1
            #v_x_km1 = v[(k-1) * 3 + 1 + (j-1) * Clen] # v{k-1} along x-axis
            #v_y_km1 = v[(k-1) * 3 + 2 + (j-1) * Clen] # v{k-1} along y-axis
            #v_z_km1 = v[(k-1) * 3 + 3 + (j-1) * Clen] # v{k-1} along z-axis
            
            v_x_k = v[k][0]  # v{k} along x-axis
            v_y_k = v[k][1]  # v{k} along y-axis
            v_z_k = v[k][2]  # v{k} along z-axis
            
            # km1 stands for k minus 1
            v_x_km1 = v[k+1][0] # v{k-1} along x-axis
            v_y_km1 = v[k+1][1] # v{k-1} along y-axis
            v_z_km1 = v[k+1][2] # v{k-1} along z-axis
            
            #END-VM


            # Computing \alpha, \beta, and \gamma values per each axis. a_0 is
            # supposed to be equal to zero
            #alfa_x  = M[0,:] * [[difference_p_x - motion_time * v_x_km1], [dv],[da]]
            #beta_x  = M[1,:] * [[difference_p_x - motion_time * v_x_km1], [dv],[da]]
            #gamma_x = M[2,:] * [[difference_p_x - motion_time * v_x_km1], [dv],[da]]

            #alfa_y  = M[0,:] * [[difference_p_y - motion_time * v_y_km1], [dv],[da]]
            #beta_y  = M[1,:] * [[difference_p_y - motion_time * v_y_km1], [dv],[da]]
            #gamma_y = M[2,:] * [[difference_p_y - motion_time * v_y_km1], [dv],[da]]

            #alfa_z  = M[0,:] * [[difference_p_z - motion_time * v_z_km1], [dv],[da]]
            #beta_z  = M[1,:] * [[difference_p_z - motion_time * v_z_km1], [dv],[da]]
            #gamma_z = M[2,:] * [[difference_p_z - motion_time * v_z_km1], [dv],[da]]
            
            alfa_x  = np.dot(M[0,:], [difference_p_x - motion_time * v_x_km1, dv, da])
            beta_x  = np.dot(M[1,:], [difference_p_x - motion_time * v_x_km1, dv, da])
            gamma_x = np.dot(M[2,:], [difference_p_x - motion_time * v_x_km1, dv, da])

            alfa_y  = np.dot(M[0,:], [difference_p_y - motion_time * v_y_km1, dv, da])
            beta_y  = np.dot(M[1,:], [difference_p_y - motion_time * v_y_km1, dv, da])
            gamma_y = np.dot(M[2,:], [difference_p_y - motion_time * v_y_km1, dv, da])

            alfa_z  = np.dot(M[0,:], [difference_p_z - motion_time * v_z_km1, dv, da])
            beta_z  = np.dot(M[1,:], [difference_p_z - motion_time * v_z_km1, dv, da])
            gamma_z = np.dot(M[2,:], [difference_p_z - motion_time * v_z_km1, dv, da])

            #END-VM


            # Velocity at the final/end point. This comes from eq. 22 of D'Andrea's
            # paper. We aim to solve the problem constraining the shape of the splines,
            # expressed in terms of position and velocity. Eventually, we want
            # to be in the final region and go there with zero velocity. v_f
            # stands for velocity final
            v_f_x = (alfa_x/24) * motion_time**4 + (beta_x/6) * motion_time**3 +(gamma_x/2) * motion_time**2 + v_x_km1

            v_f_y = (alfa_y/24)*motion_time**4 + (beta_y/6) * motion_time**3 + (gamma_y/2) * motion_time**2 + v_y_km1

            v_f_z = (alfa_z/24)*motion_time**4 + (beta_z/6) * motion_time**3 + (gamma_z/2) * motion_time**2 + v_z_km1;

            # "g" contains all the constraints to the optimization problem:
            # - constraint on the optimal acceleration (see eq.(24) D'Andrea's
            # paper) between WPs
            # - the final velocity is equal to the velocity star for the x, y, and
            # z-axis
            #
            g.append(# this is empty at the beginning (for iteration)
                #
                # See equation (24) of D'Andrea's paper. a_0 = 0
                acc_bar * difference_p_x - motion_time * acc_bar * v_x_km1) # from acceleration constraints
            g.append(acc_bar * difference_p_y - motion_time * acc_bar * v_y_km1) # from acceleration constraints
            g.append(acc_bar * difference_p_z - motion_time * acc_bar * v_z_km1) # from acceleration constraints
                # This pushes the optimization problem to have next velocity
                # values as the splines in eq.(22)
            g.append(v_x_k - v_f_x)
            g.append(v_y_k - v_f_y)
            g.append(v_z_k - v_f_z) # from velocity dynamics
            
            # Considering the constraints on the maximum acceleration in the optimization problem
            lbg.append(-max_acc * constraint_max_axis_acc[:,j])
            lbg.append(np.zeros((3,1))) # lower bounds
            ubg.append(max_acc * constraint_max_axis_acc[:,j])
            ubg.append(np.zeros(3,1)) # upper bounds

    # Casadi set up

    var = np.concatenate((p, v), axis=0)
    var_ub = np.concatenate((ubp, ubv), axis=0)
    var_lb = np.concatenate((lbp, lbv), axis=0)


    # The problem is setup in a way to consider the cost function as a
    # constraint and the object function as zero. This minimize the computation
    # time.
    if enable_boolean == 'y': # in case of boolean optimization
        g.append(-case_inspection_Ndrones(var, optimizationParameters, parameters, selection))
        lbg.append(np.finfo(np.float64).eps)
        ubg.append(math.inf)

    # Casadi settings. For better performance, you can replace "numps" solver
    # with "ma27" or "ma57". If you do not have it, change this option below
    if solver_choice == 1: # mumps
        options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4, 'max_iter', 5000, 
            'linear_solver', 'mumps', 'hessian_approximation', 'limited-memory',
            'print_level',0)) % mumps, ma27, ma57, limited-memory

    elif solver_choice == 2: # ma27
        options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4, 'max_iter', 5000, 
            'linear_solver', 'ma27', 'hessian_approximation', 'limited-memory', 
            'print_level',0))# mumps, ma27, ma57, limited-memory

    elif solver_choice == 3:# ma57
        options = struct('ipopt', struct('tol', 1e-6, 'acceptable_tol', 1e-4, 'max_iter', 5000, 
            'linear_solver', 'ma57', 'hessian_approximation', 'limited-memory', 
            'print_level',0)); # mumps, ma27, ma57, limited-memory
    else:
        # Stops execution of the file and gives control to the user's keyboard
        return

    options.print_time = false;
    options.expand     = false;
    options.verbose    = true;

    # Minimize the function "case_inspection_lausanne_Ndrones" with respect to
    # "p" and subject to "g" constraints
    if enable_boolean=='y': # in case of boolean optimization
        prob = struct('f', 0, 'x', var, 'g', g)
    else:
        prob = struct('f', case_inspection_Ndrones(var, optimizationParameters, parameters, selection), 'x', var, 'g', g)


    # Nonlinear programs. See https://web.casadi.org/docs/
    solver = nlpsol('solver', 'ipopt', prob, options);

    ## Solving the NLP

    print('Solving...');

    sol = solver('x0', var0, 'lbx', var_lb, 'ubx', var_ub,'lbg', lbg, 'ubg', ubg);

    ## Showing the solution

    time_taken =  time.time() # the time spent in solving the problem

    p_opt = full(sol.x)

    ## Plotting the solution

    print('Plotting...(press any key to start)')

    # Waiting for user input
    time.sleep(5.5) 

    # List of waypoints
    waypoints    = [None]*optimizationParameters.drones
    waypoints_LP    = [None]*optimizationParameters.drones


    # The maximum number of drones is fixed to four. When simulating more
    # device, enter a new color and line type
    # Path are indicate with dashed line
    mar = [None]*12
    mar[0]  = 'k*'  # color and line type, i.e., black and dashed
    mar[1]  = 'g*'  #  color and line type, i.e., green and dashed
    mar[2]  = 'r*'; # color and line type, i.e., red and dashed
    mar[3]  = 'b*'; # color and line type, i.e., blue and dashed
    mar[4]  = 'c*'; # color and line type, i.e., blue and dashed
    mar[5]  = 'm*'; # color and line type, i.e., blue and dashed
    mar[6]  = 'y*'; # color and line type, i.e., blue and dashed
    mar[7]  = 'r*'; # color and line type, i.e., blue and dashed
    mar[8]  = 'g*'; # color and line type, i.e., blue and dashed
    mar[9] = 'k*'; # color and line type, i.e., blue and dashed
    mar[10] = 'b*'; # color and line type, i.e., blue and dashed
    mar[11] = 'c*'; # color and line type, i.e., blue and dashed

    color_drone = [ [0.93 ,0.69, 0.13] ,   # drone 1
                    [0.49, 0.18, 0.56] ,   # drone 2
                    [0.26 ,0.72, 0.54] ,   # drone 3
                    [0.46 ,0.32, 0.74] ]  # drone 4

    # Plotting the waypoints
    plt.figure(NLP_results)

    for n in range(drones):

        waypoints[n] = np.reshape(p_opt[(n - 1) * (WPs_total + 1)*3:
            (n)*(WPs_total + 1) * 3], (3 , WPs_total + 1))
        plot3(waypoints[n][0,:], waypoints[n][1,:], waypoints[n][2,:], mar[n])

    # Evaluating the STL optimization problem with doubles as input
    negative_rob, pos_x, pos_y, pos_z = case_inspection_Ndrones(p_opt, optimizationParameters, parameters, selection)
        
    # The heading vector. It is a zeros vector and has the same dimension of
    # pos_x, pos_y and pos_z (the drone position in the 3D space)
    heading_vector = np.zeros(np.shape(pos_x)[0], optimizationParameters['drones'])

    
    # Computing the heading
    for d in range(optimizationParameters['drones']):

        WPs_cumsum = np.cumsum(WPs_sequence[d])    ###### barresi
        n_regions  = len(sequence[d])-1

        for i in range(len(pos_x)):

            # From the second point on
            if i > 0:
                heading_vector[i,d] = math.atan2(pos_y[i,d] - pos_y[i-1,d], pos_x[i,d] - pos_x[i-1,d])
            # i < 0   :  
            else :# initialization step (starting point)
                heading_vector[i,d] = math.atan2(pos_y[i,d], pos_x[i,d])

        # This part of the problem imposes that the position has to be equal
        # to the target at a certain number of waypoints, and therefore time
        
        for i in range(n_regions - 1):  # current_target
        
            String =  'parameters.I_installation.Drone_' + str(d) + '{' + str(i) + '}'
            heading_vector[eval(String),d] = goal[sequence[d][i+1]]['heading_angle']

    # Plotting the path
    for d in range(optimizationParameters['drones']):
        plot3(pos_x[:,d], pos_y[:,d], pos_z[:,d], '-.', 'color', color_drone[d,:], 'linewidth', 1.5)

    # --- Plotting the LP solution: ----------------------------------------- %
    plt.figure(NLP_results)

    for n in range(drones):

        waypoints_LP[n] = np.reshape(var0[(n - 1) * (WPs_total + 1)*3:(n)*(WPs_total + 1) * 3], (3 , WPs_total + 1));
        plot3(waypoints_LP[n][0,:], waypoints_LP[n][1,:], waypoints_LP[n][2,:], mar[n])

    # Evaluating the STL optimization problem with doubles as input
    _ , pos_x_LP, pos_y_LP, pos_z_LP = case_inspection_Ndrones(var0, optimizationParameters, parameters, selection)

    # Plotting the path
    for d in range(optimizationParameters['drones']):

        plot3(pos_x_LP[:,d], pos_y_LP[:,d], pos_z_LP[:,d], '-.', 'color',color_drone[d,:], 'linewidth', 1.5);
    # ----------------------------------------------------------------------- %
    
    plt.figure(NLP_results)

    if enable_animation =='y':

        # Load quadrotor structure
        Quad = load('Quadrotor_plotting_model.mat')

        # Scaling quadrotor dimension
        Quad['l'] = Quad['l'] * scale;
        Quad['t'] = Quad['t'] * scale;
        Quad['plot_arm'] = Quad['plot_arm'] * scale;
        Quad['plot_arm_t'] = Quad['plot_arm_t'] * scale;
        Quad['plot_prop'] = Quad['plot_prop']  * scale;
        Quad['X_armX'] = Quad['X_armX'] * scale;
        Quad['X_armY'] = Quad['X_armY'] * scale;
        Quad['X_armZ'] = Quad['X_armZ'] * scale;
        Quad['Y_armX'] = Quad['Y_armX'] * scale;
        Quad['Y_armY'] = Quad['Y_armY'] * scale;
        Quad['Y_armZ'] = Quad['Y_armZ'] * scale;
        Quad['Motor1X'] = Quad['Motor1X'] * scale;
        Quad['Motor1Y'] = Quad['Motor1Y'] * scale;
        Quad['Motor1Z'] = Quad['Motor1Z'] * scale;
        Quad['Motor2X'] = Quad['Motor2X'] * scale;
        Quad['Motor2Y'] = Quad['Motor2Y'] * scale;
        Quad['Motor2Z'] = Quad['Motor2Z']* scale;
        Quad['Motor3X'] = Quad['Motor3X'] * scale;
        Quad['Motor3Y'] = Quad['Motor3Y'] * scale;
        Quad['Motor3Z'] = Quad['Motor3Z'] * scale;
        Quad['Motor4X'] = Quad['Motor4X'] * scale;
        Quad['Motor4Y'] = Quad['Motor4Y'] * scale;
        Quad['Motor4Z'] = Quad['Motor4Z'] * scale;
    # Cell containing the quadrotors position and orientations
    for i in range(optimizationParameters['drones']):
        Quad_cell[i] = Quad

    # For the animation
    for t in range(np.shape(pos_x)[0]):

        for d in range(optimizationParameters['drones']):

            # If it is going to represent the second frame
            if t > 0 :

                # Make unvisible the previous drone position

                Quad_cell[d]['X_arm'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] , Visible='off')

                Quad_cell[d]['Y_arm'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] , Visible='off')

                Quad_cell[d]['Motor1'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] - 2 * Quad_cell[d]['t'], Visible='off')

                Quad_cell[d]['Motor2'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] - 2 * Quad_cell[d]['t'], Visible='off')

                Quad_cell[d]['Motor3'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] - 2 * Quad_cell[d]['t'], Visible='off')

                Quad_cell[d]['Motor4'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] - 2 * Quad_cell[d]['t'], Visible='off')


            # To represent the quadrotor in the 3D space
            Quad_cell[d]['X_arm'] =  mpl_toolkits.mplot3d.art3d.Patch3D(xdata=Quad_cell[d]['X_armX'], ydata=Quad_cell[d]['X_armY'], zdata= Quad_cell[d]['X_armZ'], 
                                                                        facealpha=.9, facecolor='b');

            Quad_cell[d]['Y_arm'] =  mpl_toolkits.mplot3d.art3d.Patch3D(xdata=Quad_cell[d]['Y_armX'], ydata=Quad_cell[d]['Y_armY'], zdata= Quad_cell[d]['Y_armZ'], 
                                                                        facealpha=.9, facecolor='b');

            Quad_cell[d]['Motor1'] =  mpl_toolkits.mplot3d.art3d.Patch3D(xdata=Quad_cell[d]['Motor1X'], ydata=Quad_cell[d]['Motor1Y'], zdata= Quad_cell[d]['Motor1Z'], 
                                                                        facealpha=.3, facecolor='g');

            Quad_cell[d]['Motor2'] =  mpl_toolkits.mplot3d.art3d.Patch3D(xdata=Quad_cell[d]['Motor2X'], ydata=Quad_cell[d]['Motor2Y'], zdata= Quad_cell[d]['Motor2Z'], 
                                                                        facealpha=.3, facecolor='k');

            Quad_cell[d]['Motor3'] =  mpl_toolkits.mplot3d.art3d.Patch3D(xdata=Quad_cell[d]['Motor3X'], ydata=Quad_cell[d]['Motor3Y'], zdata= Quad_cell[d]['Motor3Z'], 
                                                                        facealpha=.3, facecolor='k');

            Quad_cell[d]['Motor4'] =  mpl_toolkits.mplot3d.art3d.Patch3D(xdata=Quad_cell[d]['Motor4X'], ydata=Quad_cell[d]['Motor4Y'], zdata= Quad_cell[d]['Motor4Z'], 
                                                                        facealpha=.3, facecolor='k');

            # Quadrotor position and attitude
            Quad_cell[d]['X'] = pos_x[t,d]
            Quad_cell[d]['Y'] = pos_y[t,d]
            Quad_cell[d]['Z'] = pos_z[t,d]
            Quad_cell[d]['phi'] = 0
            Quad_cell[d]['theta'] = 0
            Quad_cell[d]['psi'] = heading_vector[t,d]

            #Plot the quadrotor
            Quad_cell[d]['Xtemp'], Quad_cell[d]['Ytemp'], Quad_cell[d]['Ztemp']= rotateBFtoGF(Quad_cell[d]['X_armX'], Quad_cell[d]['X_armY'], Quad_cell[d]['X_armZ'], Quad_cell[d]['phi'], Quad_cell[d]['theta'], Quad_cell[d]['psi'])
                
            # Update the new position within the graph            
            Quad_cell[d]['X_arm'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] )
            
            Quad_cell[d]['Xtemp'], Quad_cell[d]['Ytemp'], Quad_cell[d]['Ztemp'] = rotateBFtoGF(Quad_cell[d]['Y_armX'], Quad_cell[d]['Y_armY'], Quad_cell[d]['Y_armZ'], 
                    Quad_cell[d]['phi'], Quad_cell[d]['theta'], Quad_cell[d]['psi'])
           
            # Update the new position within the graph         
            Quad_cell[d]['Y_arm'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] )

            Quad_cell[d]['Xtemp'], Quad_cell[d]['Ytemp'], Quad_cell[d]['Ztemp'] = rotateBFtoGF(Quad_cell[d]['Motor1X'], Quad_cell[d]['Motor1Y'], Quad_cell[d]['Motor1Z'], 
                    Quad_cell[d]['phi'], Quad_cell[d]['theta'], Quad_cell[d]['psi'])
                
            # Update the new position within the graph           
            Quad_cell[d]['Motor1'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] - 2 * Quad_cell[d]['t'])
            
            Quad_cell[d]['Xtemp'], Quad_cell[d]['Ytemp'], Quad_cell[d]['Ztemp'] = rotateBFtoGF(Quad_cell[d]['Motor2X'], Quad_cell[d]['Motor2Y'], Quad_cell[d]['Motor2Z'], 
                    Quad_cell[d]['phi'], Quad_cell[d]['theta'], Quad_cell[d]['psi'])

            # Update the new position within the graph           
            Quad_cell[d]['Motor2'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] - 2 * Quad_cell[d]['t'])

            Quad_cell[d]['Xtemp'], Quad_cell[d]['Ytemp'], Quad_cell[d]['Ztemp'] = rotateBFtoGF(Quad_cell[d]['Motor3X'], Quad_cell[d]['Motor3Y'], Quad_cell[d]['Motor3Z'], 
                    Quad_cell[d]['phi'], Quad_cell[d]['theta'], Quad_cell[d]['psi'])

            # Update the new position within the graph
            Quad_cell[d]['Motor3'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] - 2 * Quad_cell[d]['t'])

            Quad_cell[d]['Xtemp'], Quad_cell[d]['Ytemp'], Quad_cell[d]['Ztemp'] = rotateBFtoGF(Quad_cell[d]['Motor4X'], Quad_cell[d]['Motor4Y'], Quad_cell[d]['Motor4Z'], 
                    Quad_cell[d]['phi'], Quad_cell[d]['theta'], Quad_cell[d]['psi'])

            # Update the new position within the graph
            Quad_cell[d]['Motor4'].set(xdata= Quad_cell[d]['Xtemp'] + Quad_cell[d]['X'], ydata=Quad_cell[d]['Ytemp'] + Quad_cell[d]['Y'], zdata=Quad_cell[d]['Ztemp']+ 
                    Quad_cell[d]['Z'] - 2 * Quad_cell[d]['t'])

        time.sleep(plotting_time)

    ## Trajectory rotation

    if enable_traj_rot=='y':
      
        print('Rotating obtained trajectories...')
        
        angle_deg = np.rad2deg(angle_rad) # degrees
        
        # Rotation angle specified as a real-valued scalar. The rotation angle is 
        # positive if the rotation is in the counter-clockwise direction when 
        # viewed by an observer looking along the z-axis towards the origin. 
        # Angle units are in degrees.
        Rz = rotz(angle_deg) # rotation matrix along z-axis     ############################################################# baresi
        
        # Vector initialization
        pos_x_rot   = np.zeros((len(pos_x), optimizationParameters['drones']))
        pos_y_rot   = np.zeros((len(pos_x), optimizationParameters['drones']))
        pos_z_rot   = np.zeros((len(pos_x), optimizationParameters['drones']))
        heading_rot = np.zeros((len(pos_x), optimizationParameters['drones']))
        
        for d in optimizationParameters['drones']:
              
            for i in range(len(pos_x)):

                temp = Rz * [pos_x[i,d], pos_y[i,d] , pos_z[i,d]];
                pos_x_rot[i,d] = temp[0] 
                pos_y_rot[i,d] = temp[1]
                pos_z_rot[i,d] = temp[2]

                
            WPs_cumsum = np.cumsum(WPs_sequence[d]);
            n_regions  = len(sequence[d])-1;
            
            for i in range(1,len(pos_x_rot)):
                
                # From the second point on
                if i > 0:
                    heading_rot[i,d] = math.atan2(pos_y_rot[i,d] - pos_y_rot[i-1,d], pos_x_rot[i,d] - pos_x_rot[i-1,d])   
                # i < 1     
                else: # initialization step (starting point)
                    heading_rot[i,d] = math.atan2(pos_y_rot[i,d], pos_x_rot[i,d])
            
            # This part of the problem imposes that the position has to be equal
            # to the target at a certain number of waypoints, and therefore time
            
            for i in range(n_regions - 1): # current_target

                String =  'parameters.I_installation.Drone_' + str(d) + '{' + str(i) + '}'
                heading_vector[eval(String),d] = goal[sequence[d][i+1]]['heading_angle'] +angle_rad
               
        heading_rot[0,:] = heading_rot[1,:]
        # Translating the trajectories
        pos_x_rot   = pos_x_rot + translation_x
        pos_y_rot   = pos_y_rot + translation_y

    ## Data saving in a txt file

    if enable_saving_txt=='y':
        
        enable_LP_trajectories = 'n'

        print('Data saving...')
        print(data_saving)


    ## Making the video animation

    print( ' ' )
    enable_video_animation = input('Do you record a video animation?y/n')
    print( ' ' )

    if enable_video_animation=='y':
        
        # Set up 3D plot to record. Figure full screen mode
        NLP_video = plt.figure('units','normalized','outerposition',[0, 0 ,1, 1])
        
        # To plot the power line mock-up
        if (selection == 1) | (selection == 2) | (selection == 3):
            plot_scenario(map, []);
        

        # Number of goals within the cell
        goal_elements     = len(goal)
        obstacle_elements = len(obstacles)

        # To plot the goal regions (they are in blue)
        for i in range(goal_elements):
            # The target point is represented as a polyhedron
            goal[i]['polyhedron'] = Polyhedron('lb', goal[i]['lb'], 'ub', goal[i]['ub'])
            plt.plot(goal[i]['polyhedron'], 'color', 'blue', 'alpha', 0.5) # plot the goals

        # To plot the starting point (they are in magenta)
        for i in range(drones):
            initial_pos['shape'] = Polyhedron('lb', initial_position[:,i]-goal[i]['ds'],'ub',initial_position[:,i]+goal[i]['ds'])
            plt.plot(initial_pos['shape'], 'color', 'magenta', 'alpha', 0.5)  # plot the obstacle

        # To plot the map dimensions(they are in green)
        data['dimension'] = map['map_dimension']
        
        # To improve the display
        marg = 0.25
        plt.axis ([data['dimension'][0]-marg  , data['dimension'][3]+marg  , data['dimension'][1]-marg ,  data['dimension'][4]+marg , data['dimension'][2]-marg  , data['dimension'][5]+marg])
        if selection != 4: # axis does not work with case 4
            ax = plt.gca()
            ax.set_aspect('equal', adjustable='box')

        view(80,10)

        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.zlabel('z [m]')

    # Plotting the path
    for d in range(optimizationParameters['drones']):

        plot3(pos_x[:,d], pos_y[:,d], pos_z[:,d], '-.', 'color',color_drone[d,:], 'linewidth', 2.0);

    # Prepare the new file.
    vidObj = VideoWriter('animation.avi');
    vidObj.Quality = 95;
    vidObj.FrameRate = 15;
    open(vidObj);
 
    # Create an animation.
    set(gca,'nextplot','replacechildren');
 
    for k in range(360):
       view(80+k,10)
       
       # Write each frame to the file.
       currFrame = getframe(NLP_video);
       writeVideo(vidObj,currFrame);
       pause(0.05)
  
#     % Close the file.
    close(vidObj);