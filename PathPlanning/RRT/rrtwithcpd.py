import rrt_with_pathsmoothing
import pytest
import numpy as np
from numpy.testing import assert_almost_equal, assert_array_almost_equal
from pycpd import rigid_registration
import random

def cpd_main():
    # ====Search Path with RRT====
    # Parameter
    obs1 = [(5,5,1)]
    for i in range(1000):
        obs1_x = random.uniform(4,6)
        obs1_y = random.uniform()
    obs2 = [(5,5,2)] # = np.dot(Y, R) + np.tile(t, (np.shape(Y)[0], 1)) nonrigid registration
    reg = rigid_registration(**{ 'X': obs2, 'Y':obs1 })
    TY, (s_reg, R_reg, t_reg) = reg.register()
    rrt = RRT(start=[0, 0], goal=[5, 10],
              randArea=[-2, 15], obstacleList=obs1) #Obstacle!
    path1 = rrt.Planning(animation=show_animation)
    path2 = np.dot(obs1, R_reg) + np.tile(t_reg, (np.shape(obs1)[0], 1))
    maxIter = 1000
    smoothedPath = PathSmoothing(path, maxIter, obstacleList)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.plot([x for (x, y) in smoothedPath], [
            y for (x, y) in smoothedPath], '-b')

        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()