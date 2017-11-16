#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True

def fileseek(fileobject):
    while(1):
        line = fileobject.readline()
        if line.startswith('#'):
            pass
        else:
            fileobject.seek(-len(line), 1) # return file pointer to first data
            break


def find_initial_time(odometry_file):
    line = odometry_file.readline()
    t0 = float(line.split()[0])
    odometry_file.seek(-len(line), 1)
    return t0


def find_initial_position(groundtruth_file):
    line = groundtruth_file.readline()
    initial_position = [float(line.split()[1]), float(line.split()[2]), float(line.split()[3])]
    groundtruth_file.seek(-len(line), 1)
    return initial_position


def parse_odometry(odometry_file):
    print "Parsing odometry data file..."
    fileseek(odometry_file)
    u = []
    while(1):
        line = odometry_file.readline()
        if not line:
            print "Parsing of odometry data complete"
            break # exit the loop if we reach EOF
        floats = map(float, line.split())
        assert (len(floats) == 3), "Odometry data found with != 3 points"
        t = floats[0] # time
        v = floats[1] # forward velocity
        w = floats[2] # angular velocity
        u.append([t, v, w])
    return u


def parse_measurement(measurement_file):
    print "Parsing measurement data file..."
    fileseek(measurement_file)
    z = []
    while(1):
        line = measurement_file.readline()
        if not line:
            print "Parsing of measurement data complete"
            break # exit the loop if we reach EOF
        vals = line.split()
        assert (len(vals) == 4), "Measurement data found with != 4 points"
        t = float(vals[0]) # time
        s = int(vals[1]) # subject
        r = float(vals[2]) # range
        b = float(vals[3]) # bearing
        z.append([t, s, r, b])
    return z


def parse_groundtruth(groundtruth_file):
    print "Parsing groundtruth data file..."
    fileseek(groundtruth_file)
    x = []
    while(1):
        line = groundtruth_file.readline()
        if not line:
            print "Parsing of groundtruth data complete"
            break # exit the loop if we reach EOF
        floats = map(float, line.split())
        assert (len(floats) == 4), "Groundtruth data found with != 4 points"
        t = floats[0] # time
        x_coord = floats[1] # x position
        y_coord = floats[2] # y position
        o = floats[3] # orientation (heading)
        x.append([t, x_coord, y_coord, o])
    return x


def setup_landmarks(landmark_file, barcode_file):
    print "Parsing landmark position data file..."
    fileseek(landmark_file)
    lm = []
    while(1):
        line = landmark_file.readline()
        if not line:
            print "Parsing of landmark position data complete"
            break # exit the loop if we reach EOF
        vals = line.split()
        assert (len(vals) == 5), "Landmark position data found with != 5 points"
        s = int(vals[0]) # subject number
        x_coord = float(vals[1]) # landmark's x coordinate
        y_coord = float(vals[2]) # landmark's y coordinate
        x_stdev = float(vals[3]) # landmark's x coordinate standard deviation
        y_stdev = float(vals[4]) # landmark's y coordinate standard deviation
        lm.append([s, x_coord, y_coord, x_stdev, y_stdev])

    print "Parsing landmark barcode data file..."
    fileseek(barcode_file)
    bc = []
    while(1):
        line = barcode_file.readline()
        if not line:
            print "Parsing of landmark barcode data complete"
            break # exit the loop if we reach EOF
        vals = map(int, line.split())
        assert (len(vals) == 2), "Landmark barcode data found with != 2 points"
        s = vals[0] # subject number
        b = vals[1] # barcode number
        bc.append([s, b])

    for line in lm:
        for subject, barcode in bc:
            if subject == line[0]:
                line[0] = barcode
                break
    return lm



if __name__ == '__main__':
    print "[WARNING] fileinit.py should not be run as main"
