#!/usr/bin/env python

import numpy as np
import scipy.stats
from params import nparticles, u_noise, z_noise
from control import motion_model
from measure import measurement_model


class particle_filter():
    '''Contains all data and methods for implementing the particle filter'''

    def __init__(self, initial_pose):
        '''initialize required variables for t=0'''
        self.M = nparticles

        # create initial particle set
        self.chi = np.empty((self.M, 3)) # instantiate particle set - M by 3 (x, y, theta)
        self.chi[:, 0] = initial_pose.x + np.random.normal(0, u_noise.x_abs, self.M)
        self.chi[:, 1] = initial_pose.y + np.random.normal(0, u_noise.y_abs, self.M)
        self.chi[:, 2] = initial_pose.theta + np.random.normal(0, u_noise.theta_abs, self.M)

        # OPTION 1: ABSOLUTE NOISE (ADDITIVE) - be sure to change in update_pose, too
        # self.sigma_control = [0.05, 0.05, 0.02] # best guess value, IN PERCENT
        self.sigma_measurement = [0.1, 0.1, 0.5] # did some trials to determine x, theta (assumed y = x)
        # self.chi[:, 0] = np.random.normal(initial_pose.t, self.sigma_control[0], self.M)
        # self.chi[:, 1] = np.random.normal(initial_pose.x, self.sigma_control[1], self.M)
        # self.chi[:, 2] = np.random.normal(initial_pose.y, self.sigma_control[2], self.M)

        # OPTION 2: RELATIVE NOISE (SCALED) - be sure to change in update_pose, too
        # self.sigma_control = [0.2, 0.2, 0.05] # best guess value, IN PERCENT
        # self.sigma_measurement = [0.2, 0.2, 0.1] # did some trials to determine x, theta (assumed y)

        # print "filter.py line 37, last particle =", self.chi[-1]
        self.w = np.ones(self.M) / float(self.M)
        # self.w = self.update_weights(initial_pose)

        # print self.w
        # print "END OF WEIGHTS, START OF X, Y, Z:"
        # print self.chi
        # position_temp = self.chi[:, 0:3]
        # mu = np.average(self.chi[:, 0:3], weights=self.w, axis=0)
        # var = np.average((position_temp - mu)**2, weights=self.w, axis=0)
        # print var
        # print self.w.shape
        # print self.chi.shape

        # self.w.fill(1./self.M)
        # self.w[0] = 0 # just messing around trying to get this shit to work
        # self.w[-1] = 2./self.M # just messing around trying to get this shit to work


    def update_pose(self, u):
        '''calculate updated pose for each particle'''
        self.chi = motion_model(u, self.chi, add_noise=True)
        for index, theta in enumerate(self.chi[:, 2]):
            if theta > np.pi: # ensure heading angle is between -pi and pi
                self.chi[index, 2] = theta - 2 * np.pi
            elif theta < -np.pi:
                self.chi[index, 2]  = theta + 2 * np.pi
        # print self.chi.max(). print self.chi.min(), print self.w.max()
        # print "filter.py line 79, last particle =", self.chi[-1]
        return self.chi


    def update_weights(self, z, LM):
        '''update weights given measurement data to an observed landmark'''
        # dist = np.sqrt((self.chi[:, 0] - pose.x)**2 + (self.chi[:, 1] - pose[2])**2)

        if z.s not in LM:
            print "measurement was to a robot, not a landmark"
            return 0

        x_diff = self.chi[:, 0] - pose.x
        y_diff = self.chi[:, 1] - pose.y
        theta_diff = (self.chi[:, 2] - pose.theta)

        x_prob = scipy.stats.norm(0, np.sqrt(self.sigma_measurement[0])).pdf(x_diff)
        y_prob = scipy.stats.norm(0, np.sqrt(self.sigma_measurement[1])).pdf(y_diff)
        theta_prob = scipy.stats.norm(0, np.sqrt(self.sigma_measurement[2])).pdf(theta_diff)

        overall_prob = x_prob * y_prob * theta_prob # calculate total probability of particle being at x, y, theta

        # particles far from a measurement will give us 0.0 for a probability
        # due to floating point limits. Once we hit zero we can never recover,
        # so add some small nonzero value to all points.
        overall_prob += 1.e-6
# WAS THIS LINE CAUSING ME PROBLEMS? SHOULDN'T IT BE =, NOT +=?
        # self.w += overall_prob
        self.w = overall_prob
        self.w /= sum(self.w) # normalize

        # determine if we should do a particle reconditioning step
        # w_var = np.average((self.w - np.average(self.w))**2)
        # print w_var
        #
        # if w_var < 10e-10:
        #     self.recondition()
            # self.update_weights(pose) # RECURSIVE, MAKE SURE THIS IS CALLED CORRECTLY

        return 1


    def resample(self):
        # weight should only be p(z_t | x_t) * w_t-1 (equation 4.37, if no resampling took place
        # NEVER RESAMPLE IF NO CONTROL STEP (i.e. x_t = x_t-1)
        # draw samples from particle set according to weights, repopulate set w/ chosen samples
        particle = np.zeros((self.M, 3))
        weight = np.zeros(self.M)

        for m in range(self.M):
            index = np.random.choice(self.M, replace=True, p=self.w)
            particle[m] = self.chi[index, 0:3]
            weight[m] = self.w[index]

        self.chi = particle
        # print "filter.py line 128, last particle =", self.chi[-1]
        self.w = weight / np.sum(weight)

        return self.w


    def extract(self):
        chi_temp = self.chi[:, 0:3]
        mu = np.average(self.chi[:, 0:3], weights=self.w, axis=0)
        var = np.average((chi_temp - mu)**2, weights=self.w, axis=0)
        return mu, var


    def recondition(self):
        # create some new, random particles (~10% of population), to help fix particle deprivation problem
        # ONLY USE THIS AS A LAST RESORT IF WE LOSE WEIGHT VARIANCE

        # if particle variance < certain value, then:
        # 1) multiply x and y components by some scalar to spread them out (centered about the mean)
        # 2) add in completely random particles
        # 3) add in particles around the measurement data

        # 1)
        mu = np.average(self.chi[:, 0:3], weights=self.w, axis=0)
        var = np.average((self.chi - mu)**2, weights=self.w, axis=0)
        spread = np.subtract(self.chi, mu)
        spread *= 5
        self.chi = np.add(spread, mu)

        # 2)
        # for i in range(0, self.M, self.M/10): # step through the set and replace every 10th particle w/ new, random value
        #     index = np.random.choice(self.M, replace=True, p=self.w)
        #     self.chi[index, 0:3] = mu * np.random.normal(1, 1.5)

        # 3)
        # neither 1 nor 2 helped, skipping 3 since I don't have much time and it's not robust

        print "ENTERED RECONDITION STEP"
        return


if __name__ == '__main__':
    print "[WARNING] filter.py should not be run as main"
