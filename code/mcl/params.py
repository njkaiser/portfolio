#!/usr/bin/env python

# USER INPUT VALUES, INITIAL DATA SETUP, ETC GOES HERE:

### FILENAMES
test_file_name = 'part_A2.dat' # open test data file
dataset = 'ds1' # choices are ds0 or ds1
odometry_file_name = dataset + '/' + dataset + '_Odometry.dat'
measurement_file_name = dataset + '/' + dataset + '_Measurement.dat'
groundtruth_file_name = dataset + '/' + dataset + '_Groundtruth.dat'
landmark_file_name = dataset + '/' + dataset + '_Landmark_Groundtruth.dat'
barcode_file_name = dataset + '/' + dataset + '_Barcodes.dat'


### MAIN PROGRAM OPTIONS:
N = 1000 # clip data for quicker development


### CONTROLLER PARAMS:
class u_noise(object):
    x = 0.001 # in percent (0.01 = 1%)
    # y = 0.005 # in percent (0.01 = 1%)
    y = 0.00000001 # trying this out for now
    theta_rel = 0.05 # in percent (0.01 = 1%)
    theta_abs = 0.005 # absolute (additive) noise variance (radians)


### PARTICLE FILTER PARAMETERS:
nparticles = 1000 # number of particles

control_x_stdev = 0.2 # stddev for particle motion in x-direction
control_y_stdev = 0.2 # stddev for particle motion in y-direction
control_theta_stdev = 0.05 # stddev for particle motion in theta

measurement_x_stdev = 0.1 # stddev for measurements in x-direction
measurement_y_stdev = 0.1 # stddev for measurements in y-direction
measurement_theta_stdev = 0.5 # stddev for measurements in theta
