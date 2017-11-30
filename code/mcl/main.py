#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
# from matplotlib import animation
from math import sqrt
from time import time#, sleep
from control import motion_model
from fileinit import parse_odometry, parse_measurement, parse_groundtruth, parse_landmarks
from measure import measurement_model, calc_expected_measurement
from plot import PathTrace, plot_particles
from params import N
from definitions import Control, ControlStamped, Pose, PoseStamped, Measurement, MeasurementStamped
from particle_filter import particle_filter
try:
    from tqdm import tqdm
except ImportError:
    print "consider installing tqdm, fool (sudo pip install tqdm)"
    tqdm = lambda iterator, **kwargs: iterator


##############################################################################
###### START SETUP ###########################################################
##############################################################################
start = time()

# data structures
U  = parse_odometry()    # odometry data
Z  = parse_measurement() # measurement data
GT = parse_groundtruth() # groundtruth data
LM = parse_landmarks()   # landmark data
N  = min(len(U), N)      # cap max iterations at length of control data

# initialize particle filter object
PF = particle_filter(GT[0])

# containers for storing data for plotting later
deadrecd_path = [GT[0]] # begin fully localized
filtered_path = [GT[0]] # begin fully localized
groundtruth_path = [GT[0]]
particles = PF.chi # seed with initial particle set

# STUFF FOR DEBUGGING
measurements = []
expected_measurements = []

j, k = 0, 0 # various indices
distance_sum = 0
angle_sum = 0

end = time()
print "setup time:", end - start

##############################################################################
####    END SETUP    #########################################################
##############################################################################
#### START MAIN LOOP #########################################################
##############################################################################

# fig, ax = plt.subplots() ### DEBUG
start = time()
deadrec_pose = GT[0]
print "running main loop for", N, "iterations..."
for i in tqdm(xrange(N)):
    ##### MOTION UPDATE #####
    deadrec_pose = motion_model(U[i], deadrec_pose)
    PF.motion_update(U[i])
    mu, var = PF.extract() # extract our belief from the particle set

    # determine which groundtruth data point is closest to our control point
    while GT[j].t <= U[i].t: j += 1;

    # store corresponding data points for later comparison
    groundtruth_path.append(GT[j])
    filtered_path.append(PoseStamped(U[i].t, *mu)) # store for plotting later
    deadrecd_path.append(deadrec_pose) # store for plotting later

    # calculate linear/angular distance traveled (only filter if distance traveled > threshold)
    distance_sum += sqrt((deadrecd_path[-1].x - deadrecd_path[-2].x)**2 + (deadrecd_path[-1].y - deadrecd_path[-2].y)**2) # running sum of linear disance traveled
    angle_sum += abs(deadrecd_path[-1].x - deadrecd_path[-2].x) # running sum of angular distance traveled

    # DEBUG
    if i%5 == 0: particles = np.vstack((particles, PF.chi));

    # DEBUG
    # if i%4 == 0:# and i > 550:
    #     particles = np.vstack((particles, PF.chi))

    # incorporate measurements up to the current control step's time
    while Z[k].t <= U[i].t and k < len(Z) - 1: k += 1;
    if k >= len(Z) - 1: continue; # there's no more measurement data to process, skip rest of loop

    # DEBUG:
    try:
        dbg = calc_expected_measurement(GT[j], LM[Z[k].s])
        expected_measurements.append(MeasurementStamped(Z[k].t, Z[k].s, dbg[0], dbg[1]))
        measurements.append(Z[k])
    except KeyError:
        pass

    ##### MEASUREMENT UPDATE #####
    if distance_sum > 0.01 or angle_sum > 0.01: # only filter if we've moved (otherwise particle variance issues)
        if PF.measurement_update(Z[k], LM): # update weights based on measurement
            PF.resample() # only resample if measurement is to a valid landmark
            distance_sum = 0 # reset to 0 once we've incorporated a measurement
            angle_sum = 0 # reset to 0 once we've incorporated a measurement


    # particles = np.vstack((particles, PF.chi))
    # fig, ax = plt.subplots()
    # plotname = 'HW0, Part A, #3 -Simulated Controller vs Ground Truth Data'
    # # plot the path of our dead reckoning
    # PathTrace(deadrecd_path, plotname, True, 'r', 'Simulated Controller')
    # # plot the position based on measurements taken
    # # PathTrace(measured, plotname, True, '0.9', 'Measured Data')
    # # plot the filter-estimated position
    # PathTrace(filtered_path, plotname, True, 'b', 'Filtered Data')
    # # plot the ground truth path
    # PathTrace(groundtruth_path, plotname, True, 'g', 'Ground Truth Data')
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
    # PathTrace(deadrecd_path, plotname, True, 'r', 'Simulated Controller')
    # PathTrace(filtered_path, plotname, True, 'b', 'Filtered Data')
    # PathTrace(groundtruth_path, plotname, True, 'g', 'Ground Truth Data')
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
    #     PathTrace(deadrecd_path, plotname, True, 'r', 'Simulated Controller')
    #     # plot the position based on measurements taken
    #     # PathTrace(measured, plotname, True, '0.9', 'Measured Data')
    #     # plot the filter-estimated position
    #     PathTrace(filtered_path, plotname, True, 'b', 'Filtered Data')
    #     # plot the ground truth path
    #     PathTrace(groundtruth_path, plotname, True, 'g', 'Ground Truth Data')
    #     plot_particles(fig, ax, particles)
    #     plt.show()


end = time()
print "elapsed time for main loop:", end - start

#################################################################################
####  END MAIN LOOP   ###########################################################
#################################################################################
####  START PLOTTING  ###########################################################
#################################################################################

# print out # of data points in each plotted dataset, for knowledge
print "there are", len(measurements), "measurements data points"
print "deadrec plot data has", len(deadrecd_path), "data points"
print "groundtruth plot data has", len(groundtruth_path), "data points"
print "filtered plot data has", len(filtered_path), "data points"


# plot groundtruth, deadreckoned, and filtered paths
fig, ax = plt.subplots()
plotname = 'HW0, Part A, #3 -Simulated Controller vs Ground Truth Data'
PathTrace(deadrecd_path, plotname, True, 'r', 'Simulated Controller')
PathTrace(filtered_path, plotname, True, 'b', 'Filtered Data')
PathTrace(groundtruth_path, plotname, True, 'g', 'Ground Truth Data')
plot_particles(fig, ax, particles)
plt.show()


# plot measurement range data vs time
plt.figure('range measurements vs time')
plt.subplot(111)
plt.scatter([z.t for z in measurements], [z.r for z in measurements], color='b', label="Measurement Actual Range")
plt.scatter([z.t for z in expected_measurements], [z.r for z in expected_measurements], color='g', label="Groundtruth Expected Range")
plt.xlabel('time [s]')
plt.ylabel('position [m]')
plt.legend()
plt.show()
plt.close()

# plot measurement bearing data vs time
plt.figure('bearing measurements vs time')
plt.subplot(111)
plt.scatter([z.t for z in measurements], [z.b for z in measurements], color='b', label="Measurement Actual Range")
plt.scatter([z.t for z in expected_measurements], [z.b for z in expected_measurements], color='g', label="Groundtruth Expected Range")
plt.xlabel('time [s]')
plt.ylabel('position [m]')
plt.legend()
plt.show()
plt.close()

assert False

# plot errors vs time
plt.figure('Errors vs Time')
plt.subplot(111)
x_error = map(lambda a: a[0]-a[1], zip([p.x for p in groundtruth_path], [p.x for p in filtered_path]))
y_error = map(lambda a: a[0]-a[1], zip([p.y for p in groundtruth_path], [p.y for p in filtered_path]))
theta_error = map(lambda a: a[0]-a[1], zip([p.theta for p in groundtruth_path], [p.theta for p in filtered_path]))
plt.plot([p.t for p in groundtruth_path], x_error, color='purple', label="Filter Error X")
plt.plot([p.t for p in groundtruth_path], y_error, color='magenta', label="Filter Error Y")
# plt.plot([p.t for p in groundtruth_path], theta_error, color='y', label="Filter Error Theta")
# plt.plot([p.t for p in deadrecd_path], [p.x for p in deadrecd_path], color='r', label="Deadrec X")
# plt.plot([p.t for p in filtered_path], [p.x for p in filtered_path], color='b', label="Filtered X")
# plt.plot([p.t for p in groundtruth_path], [p.x for p in groundtruth_path], color='g', label="Groundtruth X")
plt.title('Errors vs Time')
plt.xlabel('time [s]')
plt.ylabel('error [m]')
plt.legend()
plt.show()
plt.close()


# plot x position vs time
plt.figure('x-position vs Time')
plt.subplot(111)
plt.plot([p.t for p in deadrecd_path], [p.x for p in deadrecd_path], color='r', label="Deadrec X")
plt.plot([p.t for p in filtered_path], [p.x for p in filtered_path], color='b', label="Filtered X")
plt.plot([p.t for p in groundtruth_path], [p.x for p in groundtruth_path], color='g', label="Groundtruth X")
plt.title('x-position vs Time')
plt.xlabel('time [s]')
plt.ylabel('position [m]')
plt.legend()
plt.show()
plt.close()

# plot y position vs time
plt.figure('y-position vs Time')
plt.subplot(111)
plt.plot([p.t for p in deadrecd_path], [p.y for p in deadrecd_path], color='r', label="Deadrec Y")
plt.plot([p.t for p in filtered_path], [p.y for p in filtered_path], color='b', label="Filtered Y")
plt.plot([p.t for p in groundtruth_path], [p.y for p in groundtruth_path], color='g', label="Groundtruth Y")
plt.title('y-position vs Time')
plt.xlabel('time [s]')
plt.ylabel('position [m]')
plt.legend()
plt.show()
plt.close()

# plot heading vs time (to check if there is an inflection / negative sign missing somewhere)
plt.figure('Heading vs Time')
plt.subplot(111)
plt.plot([p.t for p in deadrecd_path], [(p.theta+np.pi)%(2*np.pi)-np.pi for p in deadrecd_path], color='r', label="Deadrec Theta")
# plt.plot([p.t for p in deadrecd_path], [p.theta for p in deadrecd_path], color='r', label="Deadrec Theta")
plt.plot([p.t for p in filtered_path], [p.theta for p in filtered_path], color='b', label="Filtered Theta")
# plt.plot([p.t for p in filtered_path], [(p.theta+np.pi)%(2*np.pi)-np.pi for p in filtered_path], color='y', label="TEST")
plt.plot([p.t for p in groundtruth_path], [p.theta for p in groundtruth_path], color='g', label="Groundtruth Theta")
plt.title('Theta vs Time')
plt.xlabel('time [s]')
plt.ylabel('theta [radians]')
plt.legend()
plt.show()
plt.close()


print "DONE"
