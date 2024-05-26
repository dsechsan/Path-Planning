import numpy as np

from src.rrt.rrt import RRT
from src.search_space.search_space import SearchSpace
# from src.utilities.plotting import Plot

class RRTPlanner(object):
    @staticmethod
    def plan(start_coord,environment):
        blocks = environment.blocks
        boundary = environment.boundary
        
        X_dimensions = np.array([(boundary[0,0], boundary[0,3]), (boundary[0,1], boundary[0,4]), (boundary[0,2], boundary[0,5])])  # dimensions of Search Space
        # obstacles
        Obstacles = []
        for k in range(blocks.shape[0]):
            Obstacles.append(tuple(blocks[k,:6]))
        Obstacles = np.array(Obstacles,dtype=tuple)
        
        x_init = tuple(start_coord)  # starting location
        x_goal = tuple(environment.goal)  # goal location

        Q = np.array([(8, 4),(4,2),(2,1)])  # length of tree edges
        r = 0.01  # length of smallest edge to check for intersection with obstacles
        max_samples = 1024*50  # max number of samples to take before timing out
        prc = 0.1  # probability of checking for a connection to goal

        # create Search Space
        X = SearchSpace(X_dimensions, Obstacles)

        # create rrt_search
        rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc)
        path = rrt.rrt_search()
        return np.array(path)
        