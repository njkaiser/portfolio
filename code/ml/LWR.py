#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import numpy as np
import matplotlib as mpl
mpl.rcParams["savefig.directory"] = "/home/njk/Courses/EECS469/HW2/figures"
import matplotlib.pyplot as plt
from itertools import chain
import math



class LWR(object):
    # TODO:
    # - allow argument to be passed in for kernel type and distance function? (implement as subclasses?)

    '''
    Python class for implementing Locally Weighted Regression machine learning algorithm
    this implementation is loosely based on the LOESS model, a kernel-based LWR algorithm
    see paper: Locally Weighted Learning [C ATKESON, A MOORE, S SCHAAL] 1997
    '''

    def __init__(self, test='not test'):#, k=30, c=1, threshold=0.001, kernel_type='Gaussian', dist_func='Euclidian'):
        # tuning parameters, constants, and other initialization stuff:

        # self.kernel_type = kernel_type
        self.memo = {} # create empty dict for memoization
        self.xval = False

        if test == 'test':
            self.k = 2000 # kernel function tuning / smoothing parameter
            # self.c1 = 1 # dummy variable for test run
            # self.c2 = 1 # dummy variable for test run
            self.weight_threshold = 0.005
            self.v_min = -999 # dummy variable for test run
            self.v_max = 999 # dummy variable for test run
            self.w_min = -999 # dummy variable for test run
            self.w_max = 999 # dummy variable for test run
        else:
            self.k = 50000 # kernel function tuning / smoothing parameter
            self.c1 = 1 # importance coefficient for linear velocity
            self.c2 = 1 # importance coefficient for angular velocity
            self.weight_threshold = 0.003 # ignore data with weight lower than this
            self.v_min = 0. # min distance step from odometry data v * dt
            self.v_max = 0.0434 # max distance step from odometry data v * dt
            self.w_min = -0.0572 # min angle step from odometry data w * dt
            self.w_max = 0.0684 # max angle step from odometry data w * dt

        print "kernel scaling factor used:", self.k


    def predict(self, query):
        # predict the output state using our algorithm, given input of query point q

        q = np.append(query, 1) # append ones to include constant factor in linear regression

        # check memo for query - if it already exists, return stored values instead of recalculating each time
        tq = tuple(q) # tupleize query so we can search for it in the memo
        # if not self.xval: # don't perform this check if we're doing cross-validation
        if tq in self.memo:
            y, w, var2 = self.memo[tq]
            return y, w, var2 # save computation by returning previously-calculated values

        B, w, Z, v = self.calc_parameters(q) # calculate regression parameters

        y = np.dot(q, B) # for weighted version

        y = list(chain.from_iterable([val.tolist()[0] for val in y]))

        # check our prediction isn't greater in magnitude than max recorded data
        # if so clip to max (these values from plotted data, outliers removed)
        if y[0] > self.v_max:
            y[0] = self.v_max
        elif y[0] < self.v_min:
            y[0] = self.v_min

        if y[1] > self.w_max:
            y[1] = self.w_max
        elif y[1] < self.w_min:
            y[1] = self.w_min

        # calculate the variance (sigma^2) value
        var2 = self.evaluate_performance(B, Z, v, w)

        # Add new entry to memo so we don't have to recalculate for each identical entry
        # if not self.xval: # don't add if we're doing cross-validation
        self.memo[tq] = (y, w, var2)

        return y, w, var2


    def calc_parameters(self, q):
        # calculate least-squares error parameters
        X = self.input_data
        y = self.output_data

        # Direct Data Weighting - see page 21 of Locally Weighted Learning [C. Atkeson]
        w = self.weight(q)
        W = np.diag(w)
        Z = np.dot(W, X)
        v = np.dot(W, y)
        term1 = np.dot(Z.T, Z)

        try:
            term2 = np.dot(np.linalg.inv(term1), Z.T)
        except:
            # print "near-singular matrix, used pseudo-inverse for query", q
            term2 = np.dot(np.linalg.pinv(term1), Z.T)

        B = np.dot(term2, v)
        return B, w, Z, v


    def weight(self, q):
        # set weights for our learning set when passed a query

        w = np.empty((len(self.input_data), 1)) # initialize array of appropriate size
        d = self.dist(q) # get distance vector from distance function
        K = np.exp(-self.k * np.square(d)) # Guassian kernel - k determines width
        w = np.sqrt(K) # weight vector

        w = [val if val > self.weight_threshold else 0 for sublist in np.matrix.tolist(w) for val in sublist] # clip points to zero if they have minimal effect on our output

        if self.xval: # if we're doing cross-validation
            w = [0 if val == 1 else val for val in w] # remove the query point from the equation by setting its weight to 0

        return w


    def dist(self, q):
        # calculate Euclidian distance from query to current line in our training data

        diffs = np.subtract(q, self.input_data) # element-wise vector differences

        # give relative importance between inputs, perhaps angular is more imporant than linear velocity for predictions?
        if len(diffs[0]) == 3: # only execute this if we're working on our real data, test data is only len of 2
            diffs[:, -2] = 1/self.c2 * diffs[:, -2]
            diffs[:, -3] = 1/self.c1 * diffs[:, -3]

        sumofsquares = np.sum(np.square(diffs), axis=1) # this may need to be tweaked, right now it's completely vanilla
        dist = np.sqrt(sumofsquares) # Euclidian distance
        dist = np.divide(dist, np.amax(dist)) # normalize so max dist = 1

        return dist


    def evaluate_performance(self, B, Z, v, w):
        # calculate variance (sigma squared) error criterion
        # see section 9.1, Locally Weighted Regression [C. Atkeson]

        term1 = []
        for i, row in enumerate(Z):
            derp = np.dot(row.tolist()[0], B).tolist()
            term1.append(derp)
        term1again = []
        for i, row in enumerate(term1):
            derp = row[0]
            term1again.append(derp)

        r = np.subtract(term1again, v)
        C = np.sum(np.square(r), axis=0)
        nLWR = np.sum(np.square(w))

        if not nLWR: # eliminate divide-by-zero cases
            var2 = [0, 0, 0] # return zeros
        else:
            var2 = C/nLWR # otherwise return actual values

        try:
            var2 = var2.tolist()[0]
        except AttributeError:
            # all this does is converts a numpy array to a list, and leaves a list alone
            # this is probably the worst abuse of an exception in the history of mankind
            # but this is due tomorrow, and ain't nobody got time for that
            pass

        return var2


    def xvalidate(self, q):
        # calculate cross-validation error
        # see section 9.3, Locally Weighted Regression [C. Atkeson]

        self.xval = True # bypass some normal steps in the process
        y, w, var2 = self.predict(q)
        self.xval = False # turn back to False in case next query is not cross-validation

        return y, w, var2


    def set_training_input(self, training_input):
        self.input_data = np.matrix(training_input) # load our 'training' data
        if np.shape(self.input_data)[0] < 2:
            self.input_data = self.input_data.T # invert if wrong shape

        self.input_data = np.c_[self.input_data, np.ones(len(self.input_data))] # append ones to include constant factor in linear regression


    def set_training_output(self, training_output):
        self.output_data = np.matrix(training_output) # load our 'training' data
        if np.shape(self.output_data)[0] < 2:
            self.output_data = self.output_data.T # invert if wrong shape

        self.output_data = np.c_[self.output_data, np.ones(len(self.output_data))] # append ones to include constant factor in linear regression



if __name__ == '__main__':
  pass

### END OF SCRIPT
