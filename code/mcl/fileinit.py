#!/usr/bin/env python

import numpy as np
# from itertools import takewhile
from definitions import Pose, ControlStamped, PoseStamped, MeasurementStamped
from params import landmark_file_name, barcode_file_name
from params import odometry_file_name, measurement_file_name, groundtruth_file_name, landmark_file_name, barcode_file_name


def parse_odometry(f=odometry_file_name):
    U = [ControlStamped(-1, *a) for a in np.loadtxt(f)]
    for i in xrange(len(U)-1):
        U[i].dt = U[i+1].t - U[i].t
    U[-1].dt = 0 # maybe not necessary, but ensures 'nice' behavior for last step
    return U


def parse_measurement(f=measurement_file_name):
    return [MeasurementStamped(*a) for a in np.loadtxt(f)] # pull control data from file


def parse_groundtruth(f=groundtruth_file_name):
    # GT = np.loadtxt(f) # pull control data from file
    return [PoseStamped(*a) for a in np.loadtxt(f)]
    # return [PoseStamped(*a) for a in takewhile(lambda b: b[0] <= end_time, np.loadtxt(f))]


def parse_landmarks(f=landmark_file_name):
    lm = np.loadtxt(f, dtype=[('ID', int), ('x', float), ('y', float), ('xvar', float), ('yvar', float)]).tolist()
    bc = np.loadtxt(barcode_file_name, dtype=int)
    LM = {}
    for i, l in enumerate(lm):
        LM[bc[i+5, 1]] = Pose(l[1], l[2], -99)
    return LM


if __name__ == '__main__':
    print "[WARNING] fileinit.py should not be run as main"
