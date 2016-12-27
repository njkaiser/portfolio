#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import numpy as np
import matplotlib.pyplot as plt
import math


class Grid(object):
    '''
    class for storing grid information:
        - grid and cell metadata
        - grid (x,y) and (i,j) locations
        - initial cell cost data
        - all class methods
    '''

    def __init__(self, size=1):
        self.set_cell_size(size)


    def set_cell_size(self, size):
        # initialize or update cell size (and update grid to match)
        self.cell_size = size
        self.cell_lower_xvals = np.arange(-2, 5, self.cell_size)
        self.cell_lower_yvals = np.arange(-6, 6, self.cell_size)
        self.grid_centroids = np.ones([len(self.cell_lower_xvals), len(self.cell_lower_yvals)])
        self.mark_occupied_cells()


    def mark_occupied_cells(self):
        # determine which cells are occupied by an obstacle
        more_obs = [[-2, 0.5], [-1.5, 0.5], [-1, 0.5], [-0.5, 0.5], [0, 0.5], [0.5, 0.5], [1, 0.5], [1.5, 0.5], [2, 0.5], [2, 0], [2, -0.5],
                    [1.5, -2.5], [2, -2.5], [2.5, -2.5], [3, -2.5], [3.5, -2.5], [4, -2.5], [4.5, -2.5], [4.7, -2.5],
                    [1.5, 4], [1.5, 3.5], [1.5, 3], [2, 3], [2.5, 3], [3, 3], [3.5, 3], [4, 3], [4.5, 3], [4.7, 3]]

        # expand our obstacle from the cell it's in to any surrounding cells within reach, if applicable
        R = 0.25 # padding radius for obstacles
        for obstacle in more_obs:
            if self.cell_size < 1: # obstacles become too large if we add padding around them with 1m x 1m cells
                for x in np.arange(obstacle[0] - R, obstacle[0] + R, self.cell_size):
                    for y in np.arange(obstacle[1] - R, obstacle[1] + R, self.cell_size):
                        i, j = self.float_to_cell_index([x, y])
                        try:
                            self.grid_centroids[i][j] = 1000 # cost for an occupied cell
                        except IndexError:
                            pass
            else:
                i, j = self.float_to_cell_index([obstacle[0], obstacle[1]])
                try:
                    self.grid_centroids[i][j] = 1000 # cost for an occupied cell
                except IndexError:
                    pass


    def float_to_cell_index(self, position):
        # determine i, j grid indices for given point x, y coordinates
        # if hasattr(position[0], "__len__"):
        #     x_index = self.cell_lower_xvals.searchsorted(position[0])
        #     y_index = self.cell_lower_yvals.searchsorted(position[1])
        #     return [x_index, y_index]
        # else:
        x_index = self.cell_lower_xvals.searchsorted(position[0])
        y_index = self.cell_lower_yvals.searchsorted(position[1])
        return [x_index, y_index]


    def cell_index_to_float(self, position):
        # determine x, y point coordinates for given i, j grid indices
        x_coord = []
        y_coord = []
        if hasattr(position[0], "__len__"):
            for i, j in position:
                x_coord.append(self.cell_size * (i + 0.5) - 2)
                y_coord.append(self.cell_size * (j + 0.5) - 6)
            return np.transpose(np.array([x_coord, y_coord]))
        else:
            x_coord.append(self.cell_size * (position[0] + 0.5) - 2)
            y_coord.append(self.cell_size * (position[1] + 0.5) - 6)
            return np.transpose(np.array([x_coord, y_coord]))



if __name__ == '__main__':
    pass

#END OF SCRIPT#
