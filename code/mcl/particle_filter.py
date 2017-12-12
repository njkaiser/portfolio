#!/usr/bin/env python

import numpy as np
import scipy.stats
from params import nparticles, u_noise, z_noise
from motion import motion_model
from measure import measurement_model


class particle_filter():
    '''Contains all data and methods for implementing the particle filter'''

    def __init__(self, initial_pose):
        '''initialize required variables for t=0'''
        # self.M = nparticles

        # create initial particle set
        self.chi = np.empty((nparticles, 3)) # instantiate particle set - M by 3 (x, y, theta)
        self.chi[:, 0] = initial_pose.x + np.random.normal(0, u_noise.x_abs, nparticles)
        self.chi[:, 1] = initial_pose.y + np.random.normal(0, u_noise.y_abs, nparticles)
        self.chi[:, 2] = initial_pose.theta + np.random.normal(0, u_noise.theta_abs, nparticles)

        # initial weights set to uniform
        self.w = np.ones(nparticles) / float(nparticles)


    def motion_update(self, u):
        '''calculate updated pose for each particle'''
        self.chi = motion_model(u, self.chi, add_noise=True)
        self.chi[:, 2] = (self.chi[:, 2] + np.pi) % (2 * np.pi) - np.pi # angle wrapping to [-pi, pi)
        return self.chi


    def measurement_update(self, z, LM):
        '''update weights given measurement data to an observed landmark'''

        try: lm = LM[z.s];
        except: return 0; # measurement was to a robot, not a landmark

        # update particle weights based on measurement data
        self.w = measurement_model(self.chi, z, LM)
        self.w /= sum(self.w)

        # determine if we should do a particle reconditioning step
        # w_var = np.average((self.w - np.average(self.w))**2)
        # print w_var
        #
        # if w_var < 10e-10:
        #     self.recondition()
            # self.measurement_update(pose) # RECURSIVE, MAKE SURE THIS IS CALLED CORRECTLY

        return 1


    def resample(self):
        '''draw samples from particle set according to weights,
        replace original set w/ chosen samples'''
        # DON'T RESAMPLE IF NO CONTROL STEP (i.e. x_t = x_t-1)
        # weight should only be p(z_t | x_t) * w_t-1 (equation 4.37) if no resampling took place

        try:
            idxs = np.random.choice(nparticles, nparticles, replace=True, p=self.w)
        except ValueError:
            print self.w
            assert False
        self.chi = self.chi[idxs, :]
        return self.w # just for debug purposes


    def extract(self):
        mu = np.average(self.chi, weights=self.w, axis=0)
        var = np.average((self.chi - mu)**2, weights=self.w, axis=0)
        return mu, var


    # def recondition(self):
    #     # create some new, random particles (~10% of population), to help fix particle deprivation problem
    #     # ONLY USE THIS AS A LAST RESORT IF WE LOSE WEIGHT VARIANCE
    #
    #     # if particle variance < certain value, then:
    #     # 1) multiply x and y components by some scalar to spread them out (centered about the mean)
    #     # 2) add in completely random particles
    #     # 3) add in particles around the measurement data
    #
    #     # 1)
    #     mu = np.average(self.chi[:, 0:3], weights=self.w, axis=0)
    #     var = np.average((self.chi - mu)**2, weights=self.w, axis=0)
    #     spread = np.subtract(self.chi, mu)
    #     spread *= 5
    #     self.chi = np.add(spread, mu)
    #
    #     # 2)
    #     # for i in range(0, nparticles, nparticles/10): # step through the set and replace every 10th particle w/ new, random value
    #     #     index = np.random.choice(nparticles, replace=True, p=self.w)
    #     #     self.chi[index, 0:3] = mu * np.random.normal(1, 1.5)
    #
    #     # 3)
    #     # neither 1 nor 2 helped, skipping 3 since I don't have much time and it's not robust
    #
    #     print "ENTERED RECONDITION STEP"
    #     return


if __name__ == '__main__':
    print "[WARNING] filter.py should not be run as main"
