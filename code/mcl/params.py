#!/usr/bin/env python

# USER INPUT VALUES, INITIAL DATA SETUP, ETC GOES HERE:

### FILENAMES:
test_file_name = 'particle_test_spiral.dat'
dataset = 'ds1' # choices are ds0 or ds1
odometry_file_name = dataset + '/' + dataset + '_Odometry.dat'
measurement_file_name = dataset + '/' + dataset + '_Measurement.dat'
groundtruth_file_name = dataset + '/' + dataset + '_Groundtruth.dat'
landmark_file_name = dataset + '/' + dataset + '_Landmark_Groundtruth.dat'
barcode_file_name = dataset + '/' + dataset + '_Barcodes.dat'


### MAIN PROGRAM OPTIONS:
N = 1000 # clip data for quicker development


### MOTION MODEL PARAMETERS:
class u_noise(object):
    x_rel = 0.120 # x noise (relative) in percent (0.01 = 1%)
    x_abs = 0.005 # x noise (absolute) in meters
    y_rel = 0.001 # y noise should mostly come from theta noise, since robot can't move sideways
    y_abs = 0.005 # y noise (absolute) in meters
    theta_rel = 0.050 # heading noise (relative) in percent (0.01 = 1%)
    theta_abs = 0.035 # heading noise (absolute) in radians


### MEASUREMENT MODEL PARAMETERS:
min_prob = 1e-12 # prevents NaNs and particle deprivation if prob = 0 due to floating point limits
class z_noise(object):
    r_rel = 0.10 # range noise (relative) in percent (0.01 = 1%)
    r_abs = 0.05 # range noise (absolute) in meters
    # b_rel = 0.10 # bearing noise (relative) in percent (0.01 = 1%)
    b_abs = 0.20 # bearing noise (absolute) in radians


### PARTICLE FILTER PARAMETERS:
nparticles = 500 # number of particles


# control_x_stdev = 0.2 # stddev for particle motion in x-direction
# control_y_stdev = 0.2 # stddev for particle motion in y-direction
# control_theta_stdev = 0.05 # stddev for particle motion in theta

# measurement_x_stdev = 0.1 # stddev for measurements in x-direction
# measurement_y_stdev = 0.1 # stddev for measurements in y-direction
# measurement_theta_stdev = 0.5 # stddev for measurements in theta
