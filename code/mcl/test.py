#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from control import motion_model
from fileinit import parse_odometry, parse_measurement, parse_groundtruth, parse_landmarks
# from measure import measurement_model, calc_expected_measurement
from plot import PathTrace, plot_particles
from definitions import Control, ControlStamped, Pose, PoseStamped, Measurement, MeasurementStamped
from particle_filter import particle_filter


# pack control input data into proper structure:
u_test = parse_odometry('particle_test_spiral.dat')

initial_pose_test = PoseStamped(0.0, 0.0, 0.0, 0.0) # hard-coded for our test run

# set up special container classes for storing x, y, theta values
gt_test = [initial_pose_test]
ctrlr_test = [initial_pose_test]
filtered_test = [initial_pose_test]
PF_test = particle_filter(initial_pose_test)

# initialize required variables
groundtruth_test = initial_pose_test
controller_test = initial_pose_test

# for debug / visualization only:
particles = np.array([0,0,0]) # seed with one data point just cuz

for i in xrange(0, len(u_test)-1):
    print '[', i, ']', u_test[i], groundtruth_test, u_test[i+1].t - u_test[i].t

    # create groundtruth data, using controller data without noise
    groundtruth_test = motion_model(u_test[i], groundtruth_test, add_noise=False) # get groundtruth coordinates
    gt_test.append(groundtruth_test) # collect points for plotting later

    # use the contoller model to dead reckon our position
    controller_test = motion_model(u_test[i], controller_test, add_noise=True) # get dead rec coordinates
    ctrlr_test.append(controller_test) # collect points for plotting later

    # filter this data and create filter points (no measurements incorporated because we have none)
    PF_test.motion_update(u_test[i])

    # PF_test.measurement_update(controller_test)
    # PF_test.resample()

    mu_test, var_test = PF_test.extract() # extract the belief from our particle set
    mu_test = PoseStamped(u_test[i].t, mu_test[0], mu_test[1], mu_test[2])
    filtered_test.append(mu_test) # collect points for plotting later

    # fig, ax = plt.fig()
    particles = np.vstack((particles, PF_test.chi))
    # plot_particles(fig, ax, PF_test.chi)

    # print PF_test.chi
    # print "particles.shape:", particles.shape
    # print "average     x:", np.mean(PF_test.chi[:, 0])
    # print "average     y:", np.mean(PF_test.chi[:, 1])
    # print "average theta:", np.mean(PF_test.chi[:, 2])



for thing in gt_test:
    print thing
print "final robot position is", gt_test[-1].x, gt_test[-1].y

fig, ax = plt.subplots()
PathTrace(gt_test, 'HW0, Part A, #2', True, 'g', 'Theoretical Groundtruth') # plot the path of our test run
PathTrace(ctrlr_test, 'HW0, Part A, #2', True, 'r', 'Controller Data (with noise)')
PathTrace(filtered_test, 'HW0, Part A #2 & Part B #7', True, 'b', 'Filter Implementation')
PF_test.chi = particles
plot_particles(fig, ax, PF_test.chi)

plt.show()

print "DONE"
