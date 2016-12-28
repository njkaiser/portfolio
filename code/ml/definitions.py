#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import numpy as np

class PositionData():
    'class to store and pass pose data for EECS469 homeworks'

    def __init__(self, initial_position):
        self.t = [initial_position[0]]
        self.x = [initial_position[1]]
        self.y = [initial_position[2]]
        self.theta = [initial_position[3]]

    def append_data(self, position):
        'function to append x, y, theta data point'
        self.t.append(position[0]) # append x coordinate
        self.x.append(position[1]) # append x coordinate
        self.y.append(position[2]) # append y coordinate
        self.theta.append(position[3]) # append theta (heading angle)

    def get_data(self):
        'function to get x, y, theta data from this program'
        retval = np.asarray([self.t, self.x, self.y, self.theta])
        retval = np.transpose(retval)
        return retval

    def get_time(self):
        return self.t

    def get_dx(self):
        return np.asarray(self.x)

    def get_dy(self):
        return np.asarray(self.y)

    def get_dtheta(self):
        return np.asarray(self.theta)

    def get_x(self):
        return np.asarray(self.x)

    def get_y(self):
        return np.asarray(self.y)

    def get_theta(self):
        return np.asarray(self.theta)



if __name__ == '__main__':
    pass
