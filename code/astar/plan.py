#!/usr/bin/env python

import sys
sys.dont_write_bytecode = True
import numpy as np
from heapq import heappush, heappop
from math import sqrt
from grid import Grid
from matplotlib.path import Path



class Astar(object):
    '''
    a (very) myopic version of online A*, with some rough forward-predicting
    path planning logic and oscillation detection
    coordinate frame is first changed from float positions to cell indices
    coordinates are changed back in run.py, prior to plotting
    '''

    def __init__(self, startxy, goalxy, grid):

        # initialize required stuff
        self.grid = grid
        self.start = self.grid.float_to_cell_index(startxy) # convert to index coordinates
        self.goal = self.grid.float_to_cell_index(goalxy) # convert to index coordinates
        self.wall_cost = 1000 # set higher than it needs to be

        # initialize path related variables
        self.position = self.start # initialize to start position
        self.parent = self.position # initialize to start position
        self.path = []
        self.parents = []

        # initialize cost function and heap related variables
        self.open_set = []
        self.closed_set = []
        self.h = self.calc_min_cost(self.position)
        self.g = 0
        self.f = self.g + self.h
        heappush(self.open_set, [self.f, self.g, self.h, self.position, self.parent])


    def go(self):
        # call this function to start looping until solution is found
        i = 0
        while not self.are_we_there_yet():
            self.next_cell()
            if not i % 10:
                print "iteration", i
            if i >=5000:
                print "EXITED LOOP DUE TO REACHING MAX # ITERATIONS"
                break
            i += 1
        print "SOLUTION FOUND AFTER", i, "ITERATIONS"
        return 0


    def next_cell(self):
        # select lowest cost cell, add current cell to closed set
        self.f, self.g, self.h, self.position, self.parent = heappop(self.open_set)
        self.closed_set.append(self.position)

        # store current cell and its parent for later use
        self.path.append(self.position)
        self.parents.append(self.parent)
        self.parent = self.position

        # now evaluate its neighbors, add them to open set, and return to loop
        self.cost_function()
        return 0


    def cost_function(self):
        # iterate through all neighboring cells
        x_index, y_index = self.position
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                position = [x_index + i, y_index + j]

                # make sure we're accounting for the true cost of diagonal moves
                cost_multiplier = 1
                if i != 0 and j != 0:
                    cost_multiplier = sqrt(2)

                # if cell is in closed set, skip
                if position in self.closed_set:
                    continue

                # if cell is in open set, skip
                if position in [row[3] for row in self.open_set]:
                    continue

                # check if we're out of bounds of the state space
                if  x_index + i < 0 or y_index + j < 0 or \
                    x_index + i >= len(self.grid.cell_lower_xvals) or \
                    y_index + j >= len(self.grid.cell_lower_yvals):
                    # print "out of bounds:", position, "(bounds are", len(self.grid.cell_lower_xvals), len(self.grid.cell_lower_yvals), ")"
                    # self.closed_set.append(position)
                    continue

                # calculate all valid neighbor costs and push to heap
                else:
                    g = self.g + self.grid.grid_centroids[x_index + i][y_index + j] * cost_multiplier
                    self.h = self.calc_min_cost(position)
                    self.f = g + self.h
                    # print position, "\tself.f:", "{0:.3f}".format(self.f), "\tg:", "{0:.3f}".format(g), "\tself.h:", "{0:.3f}".format(self.h), "self.parent:", self.parent
                    heappush(self.open_set, [self.f, g, self.h, position, self.parent])
        return 0


    def calc_min_cost(self, position):
        # estimate minimum cost from current cell to goal
        min_cost_per_cell = 1 # assumed, we might have to use cell cost data in later iterations
        diag_dist = min(abs(position[0] - self.goal[0]), abs(position[1] - self.goal[1]))
        straight_dist = max(abs(position[0] - self.goal[0]), abs(position[1] - self.goal[1])) - diag_dist
        min_cost = (straight_dist + sqrt(2) * diag_dist) * min_cost_per_cell
        return min_cost


    def are_we_there_yet(self):
        if self.position == self.goal:
            # print "YIPPEE I WIN"
            return True
        # print "BOO I DID NOT WIN"
        return False


    def get_plot_tree(self): # GET TREE
        # determine full A* tree given all points and parents
        pathxy, parentsxy = self.get_stuff()

        plot_tree = []
        codes = []
        for i, point in enumerate(pathxy):
            plot_tree.append(parentsxy[i])
            plot_tree.append(point)
            codes.append(Path.MOVETO)
            codes.append(Path.LINETO)

        return plot_tree, codes


    def get_plot_path(self):
        # determine final path solution
        pathxy, parentsxy = self.get_stuff()

        index = -1 # start at last point and work backwards
        route = [pathxy[index]]
        indices = [index]
        while(index != 0):
            coords = pathxy[index]
            route.append(coords)
            index = np.where(np.all(pathxy == parentsxy[index], axis=1))
            index = index[0].tolist()[0] # because converting between a numpy array and Python list should be this hard...
            indices.append(index)
        return route


    def get_stuff(self):
        # I don't feel like coming up with a better name right now
        pathxy = self.grid.cell_index_to_float(self.path)
        parentsxy = self.grid.cell_index_to_float(self.parents)
        return pathxy, parentsxy



if __name__ == '__main__':
    print "plan.py should not be executed as a standalone script"

#END OF SCRIPT#
