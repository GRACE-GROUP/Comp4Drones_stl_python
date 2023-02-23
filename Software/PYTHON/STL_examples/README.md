## Reach and avoid scenario

This file contains the instructions to run the MATLAB script for the reach and avoid scenario with STL specifications.

## Questions

Below are the answers to the questions posed by the MATLAB script.

* **selection** = '1'
* **enable_plot** = 'n'
* **enable_saving_txt** = 'y'
* **enable_traj_rot** = 'y'
* **enable_plot_tower** = 'y'
* **enable_animation** = 'n'
* **enable_Belta_variation** = 'y'
* **time_estimation_method** = '2'
* **solver_choice** = '1'
* **enable_boolean** = 'n'
* **selection_CVX** = 'n'

## Parameters

```
# Smooth minimum - Barrier function optimization, see https://en.wikipedia.org/wiki/Barrier_function
x_min = (-1/C)*log(sum(exp(-C*vec_x)))
# Soomth maximum - Barrier function optimization, see https://en.wikipedia.org/wiki/Barrier_function
x_max = (1/C)*log(sum(exp(C*vec_x)))

# Smooth minimum - Belta's paper
x_min = (-1/C)*log(sum(exp(-C*vec_x)));
# Smooth maximum - Belta's paper
x_max = sum(vec_x' * exp( C * vec_x) ) / ( sum( exp( C * vec_x) ) );
```

## Maps

```
# Name of the map: test_map_reach_avoid
# Comments are preceded by the # symbol

# Map dimension [lowx  lowy  lowz    upperx  uppery upperz]
map_dimension    -2.0   -2.0  0.0     2.0     2.0     2.0

# The obstacle is modeled as polyhedra within the map [lowx lowy lowz upperx uppery upperz R G B]
# This can be a matrix till 100 x 9 doubles
# See https://www.rapidtables.com/convert/color/rgb-to-hex.html

#            lowx       lowy     lowz     upperx    uppery   upperz   R    G   B
obstacles    -0.5       -0.5      0.0      0.5       0.5      1.0    255   0   0
```