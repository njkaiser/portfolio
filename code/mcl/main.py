#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from math import sqrt
from time import time, sleep
from control import motion_model
from fileinit import parse_odometry, parse_measurement, parse_groundtruth, parse_landmarks
from measure import measurement_model, calc_expected_measurement
from plot import PathTrace, plot_particles
from params import N, test_file_name
from definitions import Control, ControlStamped, Pose, PoseStamped, Measurement, MeasurementStamped
from particle_filter import particle_filter
try:
    from tqdm import tqdm
except ImportError:
    print "consider installing tqdm, fool (sudo pip install tqdm)"
    tqdm = lambda iterator, **kwargs: iterator
# import debug


# ##########################
# ##### START OF TEST ######
# ##########################

# pack control input data into proper structure:
# u_test = parse_odometry(test_file_name)
#
# initial_pose_test = PoseStamped(0.0, 0.0, 0.0, 0.0) # hard-coded for our test run
#
# # set up special container classes for storing x, y, theta values
# gt_test = [initial_pose_test]
# ctrlr_test = [initial_pose_test]
# filtered_test = [initial_pose_test]
# PF_test = particle_filter(initial_pose_test)
#
# # initialize required variables
# groundtruth_test = initial_pose_test
# controller_test = initial_pose_test
#
# # for debug / visualization only:
# particles = np.array([0,0,0]) # seed with one data point just cuz
#
# for i in xrange(0, len(u_test)-1):
#     print '[', i, ']', u_test[i], groundtruth_test, u_test[i+1].t - u_test[i].t
#
#     # create groundtruth data, using controller data without noise
#     groundtruth_test = motion_model(u_test[i], groundtruth_test, add_noise=False) # get groundtruth coordinates
#     gt_test.append(groundtruth_test) # collect points for plotting later
#
#     # use the contoller model to dead reckon our position
#     controller_test = motion_model(u_test[i], controller_test, add_noise=True) # get dead rec coordinates
#     ctrlr_test.append(controller_test) # collect points for plotting later
#
#     # filter this data and create filter points (no measurements incorporated because we have none)
#     PF_test.motion_update(u_test[i])
#
#     # PF_test.measurement_update(controller_test)
#     # PF_test.resample()
#
#     mu_test, var_test = PF_test.extract() # extract the belief from our particle set
#     mu_test = PoseStamped(u_test[i].t, mu_test[0], mu_test[1], mu_test[2])
#     filtered_test.append(mu_test) # collect points for plotting later
#
#     # fig, ax = plt.fig()
#     particles = np.vstack((particles, PF_test.chi))
#     # plot_particles(fig, ax, PF_test.chi)
#
#     # print PF_test.chi
#     # print "particles.shape:", particles.shape
#     # print "average     x:", np.mean(PF_test.chi[:, 0])
#     # print "average     y:", np.mean(PF_test.chi[:, 1])
#     # print "average theta:", np.mean(PF_test.chi[:, 2])
#
#
#
# for thing in gt_test:
#     print thing
# print "final robot position is", gt_test[-1].x, gt_test[-1].y
#
# fig, ax = plt.subplots()
# PathTrace(gt_test, 'HW0, Part A, #2', True, 'g', 'Theoretical Groundtruth') # plot the path of our test run
# PathTrace(ctrlr_test, 'HW0, Part A, #2', True, 'r', 'Controller Data (with noise)')
# PathTrace(filtered_test, 'HW0, Part A #2 & Part B #7', True, 'b', 'Filter Implementation')
# PF_test.chi = particles
# plot_particles(fig, ax, PF_test.chi)
#
# plt.show()
# assert False

##########################
######## END TEST ########
#### START MAIN LOOP #####
##########################

start = time()

# TODO: remove extraneous stuff:
# debug_file = open("debug.txt", "w")
# measurement_error = []
# heading_error = []
# dbg_range_error = []
# dbg_heading_error = []
# mu = GT[0]
# true_pose = GT[0] # just for debugging

U = parse_odometry() # odometry data
Z = parse_measurement() # measurement data
GT = parse_groundtruth() # groundtruth data
LM = parse_landmarks() # landmark data
N = min(len(U), N) # truncate max # iterations if > length of data

# containers for storing data for plotting later
deadrec_data = [GT[0]] # begin fully localized
estimated = [GT[0]] # begin fully localized
groundtruth = [GT[0]]
measurements = []
expected_measurements = []

# TODO: remove these and just use deadrec_data[-1], etc?
pose = GT[0]
deadrec_pose = GT[0]
filtered_pose = GT[0]
PF = particle_filter(GT[0])
particles = np.array([GT[0].x, GT[0].y, GT[0].theta]) # seed with one data point for vstack


j, k, m = 0, 0, 0 # various indices
distance_sum = 0
angle_sum = 0

end = time()
print "setup time:", end - start

# fig, ax = plt.subplots() ### DEBUG

start = time()
print "running main loop for", N, "iterations..."
# for i in xrange(N):
for i in tqdm(xrange(N)):

    # MOTION UPDATE
    pose = motion_model(U[i], pose)
    PF.motion_update(U[i])
    # if i%4 == 0:# and i > 550:
    #     particles = np.vstack((particles, PF.chi))
    mu, var = PF.extract() # extract our belief from the particle set
    estimated.append(PoseStamped(U[i].t, *mu)) # collect these points for plotting later


    deadrec_data.append(pose) # collect these points for plotting later
    distance_sum += sqrt((deadrec_data[-1].x - deadrec_data[-2].x)**2 + (deadrec_data[-1].y - deadrec_data[-2].y)**2) # running sum of linear disance traveled
    angle_sum += abs(deadrec_data[-1].x - deadrec_data[-2].x) # running sum of angular disance traveled

    if i%100 == 0:# and i>500:
        particles = np.vstack((particles, PF.chi))
        # print "m, i, len(GT), len(U):", m, i, len(GT), len(U)
        # print "particles.shape:", particles.shape

    # determine which groundtruth data point is closest to our control point
    while GT[j].t <= U[i].t:
        j += 1
    groundtruth.append(GT[j])


    # run this portion if we have a measurement to incorporate
    while Z[k].t <= U[i].t: # incorporate measurements up to the current control step's time
        if k == len(Z) - 1: # there's no more measurement data to use, exit loop
            break
        k += 1 # increment to next measurement


    # DEBUG
    if k != len(Z) - 1: # there's still measurement data to process

        try:
            dbg = calc_expected_measurement(GT[m], LM[Z[k].s])
            expected_measurements.append(MeasurementStamped(Z[k].t, Z[k].s, dbg[0], dbg[1]))
            measurements.append(Z[k])
        except KeyError:
            # print "why am I broken?", Z[k].s
            # for key in LM:
            #     print key, LM[key]
            # assert False
            pass

        if distance_sum > 0.01 or angle_sum > 0.01: # only filter if we've moved, otherwise we might have particle variance issues
            # check if measurement is to a valid landmark before trying to use it...
            if not PF.measurement_update(Z[k], LM): # update weights based on measurement
                continue
            else:
                w_trblsht = PF.resample() # resample
                distance_sum = 0 # reset to 0 for the next loop
                angle_sum = 0 # reset to 0 for the next loop








    # particles = np.vstack((particles, PF.chi))
    # fig, ax = plt.subplots()
    # plotname = 'HW0, Part A, #3 -Simulated Controller vs Ground Truth Data'
    # # plot the path of our dead reckoning
    # PathTrace(deadrec_data, plotname, True, 'r', 'Simulated Controller')
    # # plot the position based on measurements taken
    # # PathTrace(measured, plotname, True, '0.9', 'Measured Data')
    # # plot the filter-estimated position
    # PathTrace(estimated, plotname, True, 'b', 'Filtered Data')
    # # plot the ground truth path
    # PathTrace(groundtruth, plotname, True, 'g', 'Ground Truth Data')
    # plot_particles(fig, ax, particles)
    # # plt.show()
    # plt.show(block=False)
    # sleep(0.1)
    # plt.close()


    # ##### BEGIN ANIMATION
    # particles = np.vstack((particles, PF.chi))
    # fig = plt.figure() # make figure
    # plotname = 'HW0, Part A, #3 -Simulated Controller vs Ground Truth Data'
    # img = plt.imshow(slices[0], cmap='gray')#, vmin=0, vmax=255)
    # PathTrace(deadrec_data, plotname, True, 'r', 'Simulated Controller')
    # PathTrace(estimated, plotname, True, 'b', 'Filtered Data')
    # PathTrace(groundtruth, plotname, True, 'g', 'Ground Truth Data')
    # plot_particles(fig, ax, particles)
    # plt.show()
    #
    #
    # def updatefig(iii): # callback function for FuncAnimation()
    #     img.set_array(slices[iii])
    #     return [img]
    #
    # anim = animation.FuncAnimation(fig, updatefig, frames=range(num_slices), interval=1, blit=True, repeat=False)
    # fig.show()
    #
    # Writer = animation.writers['ffmpeg']
    # writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
    # anim.save(str(uid)+'.mp4', writer=writer)
    # ##### END ANIMATION


    # if i%2 == 0 and i > 550:
    #     particles = np.vstack((particles, PF.chi))
    #     fig, ax = plt.subplots()
    #     plotname = 'HW0, Part A, #3 -Simulated Controller vs Ground Truth Data'
    #     # plot the path of our dead reckoning
    #     PathTrace(deadrec_data, plotname, True, 'r', 'Simulated Controller')
    #     # plot the position based on measurements taken
    #     # PathTrace(measured, plotname, True, '0.9', 'Measured Data')
    #     # plot the filter-estimated position
    #     PathTrace(estimated, plotname, True, 'b', 'Filtered Data')
    #     # plot the ground truth path
    #     PathTrace(groundtruth, plotname, True, 'g', 'Ground Truth Data')
    #     plot_particles(fig, ax, particles)
    #     plt.show()


end = time()
print "elapsed time for main loop:", end - start

###########################
###### END MAIN LOOP ######
##### START PLOTTING ######
###########################

# PLOTTING AND DATA OUTPUT
# print out # of data points in each plotted dataset, for knowledge
print "deadrec plot data has ", len(deadrec_data), " elements"
print "measurement plot data has ", len(measurements), " elements (", len(expected_measurements), ")"
print "groundtruth plot data has ", len(groundtruth), " elements"
print "filtered plot data has ", len(estimated), " elements"


fig, ax = plt.subplots()
plotname = 'HW0, Part A, #3 -Simulated Controller vs Ground Truth Data'
PathTrace(deadrec_data, plotname, True, 'r', 'Simulated Controller')
PathTrace(estimated, plotname, True, 'b', 'Filtered Data')
PathTrace(groundtruth, plotname, True, 'g', 'Ground Truth Data')
plot_particles(fig, ax, particles)
plt.show()


# # plot measurement range data vs time
# plt.figure('range measurements vs time')
# plt.subplot(111)
# plt.scatter([z.t for z in measurements], [z.r for z in measurements], color='b', label="Measurement Actual Range")
# plt.scatter([z.t for z in expected_measurements], [z.r for z in expected_measurements], color='g', label="Groundtruth Expected Range")
# # plt.plot([p.t for p in deadrec_data], [p.x for p in deadrec_data], color='b', label="Actual Measurement")
# # plt.plot([p.t for p in estimated], [p.x for p in estimated], color='g', label="Groundtruth Measurement")
# # plt.title('x-position vs Time')
# plt.xlabel('time [s]')
# plt.ylabel('position [m]')
# plt.legend()
# plt.show()
# plt.close()
#
#
# # plot measurement range data vs time
# plt.figure('bearing measurements vs time')
# plt.subplot(111)
# plt.scatter([z.t for z in measurements], [z.b for z in measurements], color='b', label="Measurement Actual Range")
# plt.scatter([z.t for z in expected_measurements], [z.b for z in expected_measurements], color='g', label="Groundtruth Expected Range")
# # plt.plot([p.t for p in deadrec_data], [p.x for p in deadrec_data], color='b', label="Actual Measurement")
# # plt.plot([p.t for p in estimated], [p.x for p in estimated], color='g', label="Groundtruth Measurement")
# # plt.title('x-position vs Time')
# plt.xlabel('time [s]')
# plt.ylabel('position [m]')
# plt.legend()
# plt.show()
# plt.close()


# plot errors vs time
plt.figure('Errors vs Time')
plt.subplot(111)
x_error = map(lambda a: a[0]-a[1], zip([p.x for p in groundtruth], [p.x for p in estimated]))
y_error = map(lambda a: a[0]-a[1], zip([p.y for p in groundtruth], [p.y for p in estimated]))
theta_error = map(lambda a: a[0]-a[1], zip([p.theta for p in groundtruth], [p.theta for p in estimated]))
plt.plot([p.t for p in groundtruth], x_error, color='purple', label="Filter Error X")
plt.plot([p.t for p in groundtruth], y_error, color='magenta', label="Filter Error Y")
# plt.plot([p.t for p in groundtruth], theta_error, color='y', label="Filter Error Theta")
# plt.plot([p.t for p in deadrec_data], [p.x for p in deadrec_data], color='r', label="Deadrec X")
# plt.plot([p.t for p in estimated], [p.x for p in estimated], color='b', label="Filtered X")
# plt.plot([p.t for p in groundtruth], [p.x for p in groundtruth], color='g', label="Groundtruth X")
plt.title('Errors vs Time')
plt.xlabel('time [s]')
plt.ylabel('error [m]')
plt.legend()
plt.show()
plt.close()

# plot x position vs time
plt.figure('x-position vs Time')
plt.subplot(111)
plt.plot([p.t for p in deadrec_data], [p.x for p in deadrec_data], color='r', label="Deadrec X")
plt.plot([p.t for p in estimated], [p.x for p in estimated], color='b', label="Filtered X")
plt.plot([p.t for p in groundtruth], [p.x for p in groundtruth], color='g', label="Groundtruth X")
plt.title('x-position vs Time')
plt.xlabel('time [s]')
plt.ylabel('position [m]')
plt.legend()
plt.show()
plt.close()

# plot y position vs time
plt.figure('y-position vs Time')
plt.subplot(111)
plt.plot([p.t for p in deadrec_data], [p.y for p in deadrec_data], color='r', label="Deadrec Y")
plt.plot([p.t for p in estimated], [p.y for p in estimated], color='b', label="Filtered Y")
plt.plot([p.t for p in groundtruth], [p.y for p in groundtruth], color='g', label="Groundtruth Y")
plt.title('y-position vs Time')
plt.xlabel('time [s]')
plt.ylabel('position [m]')
plt.legend()
plt.show()
plt.close()

# plot heading vs time (to check if there is an inflection / negative sign missing somewhere)
plt.figure('Heading vs Time')
plt.subplot(111)
plt.plot([p.t for p in deadrec_data], [(p.theta+np.pi)%(2*np.pi)-np.pi for p in deadrec_data], color='r', label="Deadrec Theta")
# plt.plot([p.t for p in deadrec_data], [p.theta for p in deadrec_data], color='r', label="Deadrec Theta")
plt.plot([p.t for p in estimated], [p.theta for p in estimated], color='b', label="Filtered Theta")
# plt.plot([p.t for p in estimated], [(p.theta+np.pi)%(2*np.pi)-np.pi for p in estimated], color='y', label="TEST")
plt.plot([p.t for p in groundtruth], [p.theta for p in groundtruth], color='g', label="Groundtruth Theta")

plt.title('Theta vs Time')
plt.xlabel('time [s]')
plt.ylabel('theta [radians]')
plt.legend()
plt.show()
plt.close()


print "DONE"
#END OF SCRIPT#
