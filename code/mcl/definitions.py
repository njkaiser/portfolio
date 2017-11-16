#!/usr/bin/env python
import numpy as np

class Pose(object):
    def __init__(self, x, y, theta):
        self.x = x # x coordinate
        self.y = y # y coordinate
        self.theta = theta # heading angle

    def __str__(self):
        return '(' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.theta) + ')'


class PoseStamped(Pose):
    def __init__(self, t, x, y, theta):
        Pose.__init__(self, x, y, theta)
        self.t = t # time stamp

    def __str__(self):
        return str(self.t) + ' (' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.theta) + ')'


class Control(object):
    def __init__(self, dt, v, w):
        self.v = v # forward velocity
        self.w = w # angular velocity
        self.dt = dt # time control is applied for

    def __str__(self):
        return '(' + str(self.v) + ', ' + str(self.w) + ') [' + str(self.dt) + ']'


class ControlStamped(Control):
    def __init__(self, dt, t, v, w):
        Control.__init__(self, dt, v, w)
        self.t = t # time stamp

    def __str__(self):
        return str(self.t) + ' (' + str(self.v) + ', ' + str(self.w) + ') [' + str(self.dt) + ']'


class Measurement(object):
    def __init__(self, s, r, b):
        self.s = int(s) # subject number
        self.r = r # range [meters]
        self.b = b # bearing [rads]

    def __str__(self):
        return str(self.s) + ' (' + str(self.r) + ', ' + str(self.b) + ')'


class MeasurementStamped(Measurement):
    def __init__(self, t, s, r, b):
        Measurement.__init__(self, s, r, b)
        self.t = t # time stamp

    def __str__(self):
        return str(self.t) + ' - ' + str(self.s) + ' (' + str(self.r) + ', ' + str(self.b) + ')'



if __name__ == '__main__':
    print "[WARNING] definitions.py should not be run as main"
