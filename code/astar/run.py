#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import numpy as np
from grid import Grid
from plan import Astar

import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import matplotlib.patches as patches
from matplotlib.path import Path


def main():
    ### initialize grid to desired size
    grid = Grid(0.2)

    # set start and goal positions
    start = [-1.55, -5.5]
    goal = [4.5, 5.5]

    # initialize astar object with grid, start, and goal
    astar = Astar(start, goal, grid)

    # run the algorithm
    astar.go()

    # plot the results
    plot(grid, astar)
    plt.show()


def plot(grid, astar_instance):
    # initial setup
    fig, ax = plt.subplots()

    # plot obstacles as binary grid background
    plt.imshow(grid.grid_centroids.transpose(), cmap=plt.cm.binary, interpolation='none', origin='lower', extent=[-2, 5, -6, 6])

    # change plotted grid base currently set to 1
    loc = plticker.MultipleLocator(base=1)
    ax.xaxis.set_major_locator(loc)
    ax.yaxis.set_major_locator(loc)
    ax.grid(which='major', axis='both')
    plt.grid(True, color='blue', linestyle='--')

    # set our x and y axis limits and label them
    plt.axis([-2, 5, -6, 6]) # world size: x = [-2, 5], y = [-6, 6]
    plt.xlabel('x position [m]')
    plt.ylabel('y position [m]')

    # convert our input floating point values to be the center of closest grid cell
    startij = astar_instance.start
    goalij = astar_instance.goal
    startxy = astar_instance.grid.cell_index_to_float(startij)[0]
    goalxy = astar_instance.grid.cell_index_to_float(goalij)[0]

    # set up markers for start, goal, and obstacles
    plt.scatter(startxy[0], startxy[1], s=300, color='orange', alpha=0.7, label='Start', zorder=99) # plot start point
    plt.scatter(goalxy[0], goalxy[1], s=300, color='green', alpha=1, label='Goal', zorder=99) # plot goal point

    # plot full A* tree, all expanded nodes and how they connect to parents
    # plot_tree, codes = astar_instance.get_plot_tree()
    # path = Path(plot_tree, codes)
    # patch = patches.PathPatch(path, color='0.8')
    # ax.add_patch(patch)

    # plot final A* solution
    route = astar_instance.get_plot_path()
    route_x = [row[0] for row in route]
    route_y = [row[1] for row in route]
    plt.plot(route_x, route_y, lw=3, color='orange', alpha=0.7)
    # plt.axis('off')



if __name__ == '__main__':
    main()
