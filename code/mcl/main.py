#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt


# project includes
from control import motion_model
from fileinit import parse_odometry, parse_measurement, parse_groundtruth, parse_landmarks
from measure import measurement_model
from plot import PathTrace, plot_particles
import debug
from params import N, test_file_name#, odometry_file_name, measurement_file_name, groundtruth_file_name, landmark_file_name, barcode_file_name
from definitions import Control, ControlStamped, Pose, PoseStamped, Measurement, MeasurementStamped#, PositionData
from particle_filter import particle_filter
# np.set_printoptions(threshold=np.nan) # prints entire numpy array


# def main():

# # ##########################
# # ##### START OF TEST ######
# # ##########################
#
# # pack control input data into proper structure:
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
#     PF_test.update_pose(u_test[i])
#
#     # PF_test.update_weights(controller_test)
#     # PF_test.resample()
#
#     mu_test, var_test = PF_test.extract() # extract the belief from our particle set
#     mu_test = PoseStamped(u_test[i].t, mu_test[0], mu_test[1], mu_test[2])
#     filtered_test.append(mu_test) # collect points for plotting later
#
#     # fig, ax = plt.fig()
#     particles = np.vstack((particles, PF_test.chi))
#     # plot_particles(fig, ax, PF_test)
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
# plot_particles(fig, ax, PF_test)
#
# plt.show()
# assert False

##########################
######## END TEST ########
#### START MAIN LOOP #####
##########################

# TODO: remove extraneous stuff:
# debug_file = open("debug.txt", "w")
# measurement_error = []
# heading_error = []
# dbg_range_error = []
# dbg_heading_error = []
# mu = GT[0]
particles = np.array([0,0,0]) # seed with one data point just cuz

U = parse_odometry() # odometry data
N = N if len(U) > N else len(U) # truncate max # iterations if > length of data
Z = parse_measurement() # measurement data
GT = parse_groundtruth(U[N].t) # groundtruth data
LM = parse_landmarks() # landmark data
print "total number of control iterations:", N

# containers for storing data for plotting later
deadrec_data = [GT[0]] # begin fully localized
measured = [GT[0]] # begin fully localized
estimated = [GT[0]] # begin fully localized
groundtruth = [GT[0]]

# TODO: remove these and just use deadrec_data[-1], etc?
pose = GT[0]
deadrec_pose = GT[0]
filtered_pose = GT[0]

# initialize required variables
PF = particle_filter(GT[0])

true_pose = GT[0] # just for debugging
j, k, m = 0, 0, 0 # various indices
distance_cum = 0
angle_cum = 0

for i in xrange(N):
    # use the contoller model to dead reckon our pose
    pose = motion_model(U[i], pose)
    deadrec_data.append(pose) # collect these points for plotting later

    distance_cum += sqrt((deadrec_data[-1].x - deadrec_data[-2].x)**2 + (deadrec_data[-1].y - deadrec_data[-2].y)**2) # running sum of linear disance traveled
    angle_cum += abs(deadrec_data[-1].x - deadrec_data[-2].x) # running sum of angular disance traveled

    PF.update_pose(U[i])

    # determine which groundtruth data point is closest to our control point
    while GT[m].t <= U[i].t:
        m += 1
    # true_pose = [ x[m][0], x[m][1], x[m][2], x[m][3] ]
    groundtruth.append(GT[m])

    # TODO: should this only incorporate the LAST valid measurement, in case a bunch of measurements are applied in succession and kill our particle variance?
    # run this portion if we have a measurement to incorporate
    while Z[j].t <= U[i].t: # incorporate measurements up to the current control step's time
        print "measured stuff"
        print "dist/angle:", distance_cum, angle_cum

        if j == len(Z) - 1: # there's no more measurement data to use, exit loop
            raise Exception("I shouldn't be here until the very end?")
            break

        j += 1 # increment to next measurement


        # try:
        #     pose = measurement_model(pose, Z[j], LM)
        # except LookupError: # if there's no data for the landmark subject #
        #     # print "Data not found for landmark #", z[j][1], " (it's probably a robot)"
        #     j += 1
        #     print "every time?"
        #     continue # we skip the rest of the loop and start again
        #
        # measured.append(pose) # collect these points for plotting later

        # debug_measured_pose, r, b, r_real, b_real = debug.determine_sensor_model(true_pose, z[j], lm)
        # dbg_range_error.append(r_error)
        # dbg_heading_error.append(b_error)
        # debug_file.write("Landmark #: %i \t Measured Range: %f \t Measure Heading: %f \t Real Range: %f \t Real Heading: %f \n" % (z[j][1], r, b, r_real, b_real))
        # debug.determine_sensor_error(debug_measured_pose, true_pose, z[j], lm)

        # measured.append(measured_position) # collect these points for plotting later

        # just collecting some data for development / troubleshooting:
        # measurement_error.append(np.sqrt((position[1]-true_pose[1])**2 + (position[2] - true_pose[2])**2))
        # heading_error.append(position[3]-true_pose[3])

    if distance_cum > 0.01 or angle_cum > 0.01: # only filter if we've moved, otherwise we'll have particle variance issues
        print "moved"
        # check if measurement is to a valid landmark before trying to use it...
        if not PF.update_weights(Z[j], LM): # update weights based on measurement
            continue

        w_trblsht = PF.resample() # resample
        mu, var = PF.extract() # extract our belief from the particle set
        mu = [U[i].t, mu[0], mu[1], mu[2]] # prepend time for easy plotting later



        estimated.append(mu) # collect these points for plotting later

        distance_cum = 0 # reset to 0 for the next loop
        angle_cum = 0 # reset to 0 for the next loop

        # THIS SECTION PLOTS HISTOGRAMS FOR PARTICLE VALUES AND WEIGHTS EVERY 100 ITERATIONS
        # IT'S NOT PART OF THE MAIN CODE, JUST TO HELP WITH DEBUGGING
        # if i >= 100 * k:
        #     # print "  ITER #:", i
        #     # print "PARTICLE:", np.average(chi_trblsht, axis=0), chi_trblsht.min(), chi_trblsht.max()
        #     # print "  WEIGHT:", np.average(w_trblsht, axis=0), w_trblsht.min(), w_trblsht.max()
        #
        #     # troubleshooting
        #     plt.figure(i)
        #     plt.hist(w_trblsht*1000, bins=20, alpha=0.5, color='r', label='weight (*10^3)')#, range=[0,0.01])
        #     plt.hist(chi_trblsht[:, 0], bins=20, alpha=0.5, color='b', label='particle x')#, range=[0.5,1.5])
        #     plt.hist(chi_trblsht[:, 1], bins=20, alpha=0.5, color='g', label='particle y')
        #     plt.hist(chi_trblsht[:, 2], bins=20, alpha=0.5, color='m', label='particle theta')
        #     plt.legend(loc='upper left')
        #     plt.show()
        #     plt.close()
        #     k += 1


    # these 2 lines just allow me to print the iteration # without taking a lot of space in the terminal window
    # sys.stdout.flush()
    # sys.stdout.write('%s %d\r' % ("main loop iteration: ", i))
    if i%100 == 0:
        print "main loop iteration", i
        particles = np.vstack((particles, PF.chi))
        print "particles.shape:", particles.shape




###########################
###### END MAIN LOOP ######
##### START PLOTTING ######
###########################

# PLOTTING AND DATA OUTPUT
# print out # of data points in each plotted dataset, for knowledge
print "deadrec plot data has ", len(deadrec_data), " elements"
print "measured plot data has ", len(measured), " elements"
print "groundtruth plot data has ", len(groundtruth), " elements"
print "filtered plot data has ", len(estimated), " elements"

fig, ax = plt.subplots()
plotname = 'HW0, Part A, #3 -Simulated Controller vs Ground Truth Data'
# plot the path of our dead reckoning
PathTrace(deadrec_data, plotname, True, 'r', 'Simulated Controller')
# plot the position based on measurements taken
# PathTrace(measured, plotname, True, '0.9', 'Measured Data')
# plot the filter-estimated position
PathTrace(estimated, plotname, True, 'b', 'Filtered Data')
# plot the ground truth path
PathTrace(groundtruth, plotname, True, 'g', 'Ground Truth Data')
PF.chi = particles
plot_particles(fig, ax, PF)
plt.show()
assert False


# plot x position vs time, with measurement data
plt.figure('x-position vs Time')
plt.subplot(111)
plt.plot(estimated.t, estimated.x, color='b', label="Filtered X")
plt.plot(groundtruth.t, groundtruth.x, color='g', label="Groundtruth X")
plt.plot(measured.t, measured.x, color='y', marker='.', label="Measured X")
plt.title('x-position vs Time')
plt.xlabel('time [s]')
plt.ylabel('position [m]')
plt.legend()
plt.show()
plt.close()

# plot y position vs time, with measurement data
plt.figure('y-position vs Time')
plt.subplot(111)
plt.plot(estimated.t, estimated.y, color='b', label="Filtered Y")
plt.plot(groundtruth.t, groundtruth.y, color='g', label="Groundtruth Y")
plt.plot(measured.t, measured.y, color='y', marker='.', label="Measured Y")
plt.title('y-position vs Time')
plt.xlabel('time [s]')
plt.ylabel('position [m]')
plt.legend()
plt.show()
plt.close()

# plot heading vs time (want to check to see if there is an inflection / negative sign I'm messing up somewhere)
plt.figure('Heading vs Time')
plt.subplot(111)
plt.plot(estimated.t, estimated.theta, color='b', label="Filtered Theta")
plt.plot(groundtruth.t, groundtruth.theta, color='g', label="Groundtruth Theta")
plt.plot(measured.t, measured.theta, color='y', marker='.', label="Measured Theta")
plt.title('Theta vs Time')
plt.xlabel('time [s]')
plt.ylabel('theta [radians]')
plt.legend()
plt.show()
plt.close()


# THIS SECTION PLOTS HISTOGRAMS FOR VARIOUS MEASUREMENT DISCREPANCIES
# IT'S NOT PART OF THE MAIN CODE, JUST TO HELP WITH DEBUGGING
# plot histogram of measurement error
# print len(measurement_error)
# plt.figure(1)
# plt.hist(measurement_error, bins=20, range=[0,0.5])
# plt.title("Measurement error (sqrt(x^2 + y^2) difference between groundtruth)")
# plt.xlabel("Error (meters)")
# plt.show()
# plt.close()
#
# print len(heading_error)
# plt.figure(2)
# plt.hist(heading_error, bins=20)
# plt.title("Heading error (difference between measured & groundtruth)")
# # plt.axis([-0.001, 0.001, 0, 400])
# plt.xlabel("Error (radians)")
# plt.show()

# print len(dbg_range_error)
# plt.figure(1)
# plt.hist(dbg_range_error, bins=20)#, range=[0,0.5])
# # plt.title("Measurement error (sqrt(x^2 + y^2) difference between groundtruth)")
# plt.xlabel("Range Error (%)")
# plt.show()
# plt.close()
#
# print len(dbg_heading_error)
# plt.figure(2)
# plt.hist(heading_error, bins=20)
# plt.title("Heading error (%)")
# # plt.axis([-0.001, 0.001, 0, 400])
# plt.xlabel("Error (radians)")
# plt.show()


# CLEANUP
# odometry_file.close()
# measurement_file.close()
# groundtruth_file.close()
# landmark_file.close()
# barcode_file.close()
# debug_file.close()
print "DONE"


# if __name__ == '__main__':
#     main()

#END OF SCRIPT#
