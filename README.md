# Path Planning in 3D environments

## Overview
In this assignment, you will implement and compare the performance of search-based and sampling-based motion planning algorithms on several 3-D environments.

### 1. main.py
This file contains examples of how to load and display the environments and how to call a motion planner and plot the planned path. Feel free to modify this file to fit your specific goals for the project. In particular, you should certainly replace Line 104 with a call to a function which checks whether the planned path intersects the boundary or any of the blocks in the environment.

### 2. Planner.py
This file contains an implementation of a baseline planner. The baseline planner gets stuck in complex environments and is not very careful with collision checking. 

### 3. astar_new.py
This file contains a class defining a node for the A* algorithm as well as the algorithm itself. It also contains the collision checking function.

### 4. RRT.py
This file contains the implementation of RRT algorithm.

### 5. maps
This folder contains 7 test environments described via a rectangular outer boundary and a list of rectangular obstacles. The start and goal points for each environment are specified in main.py.

### 6. Results
The results of our algorithms are present in the directories with the name of the environment, i.e., cube, flappybird, maze etc... They contain both the RRT and A* results.

<div style="display: flex; justify-content: space-between;">
    <img src="https://github.com/dsechsan/Path-Planning/blob/5438cbd8d1b75516d1b4f2c219be495078c67156/starter_code/maze/maze5.png" alt="Maze A*" width="600" height="600">
    <img src="https://github.com/dsechsan/Path-Planning/blob/5438cbd8d1b75516d1b4f2c219be495078c67156/starter_code/maze/mazerrt.png" alt="Maze RRT" width="600" height="400">
</div>
