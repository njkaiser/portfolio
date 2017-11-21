#!/usr/bin/env python
import numpy as np
from definitions import PoseStamped


def update_pose(pose, z, lm, add_noise=False):
    # determine our pose based on measurement data

    # s = z[1] # subject number
    # r = z[2] # range (distance from robot to landmark)
    # b = z[3] # bearing (angle between robot forward and landmark)
    # theta = pose.theta # need to know which direction we think the robot is pointing

    s_index = None
    for count, row in enumerate(lm):
        if row[0] == z.s:
            s_index = count
            break
    if s_index is None or s_index in [5, 14, 41, 32, 23]:
        raise LookupError("no landmark")

    lm_x = lm[s_index][1]
    lm_y = lm[s_index][2]

    measured_x = lm_x - z.r * np.cos(pose.theta + z.b)
    measured_y = lm_y - z.r * np.sin(pose.theta + z.b)
    measured_theta = np.arctan2((lm_y - measured_y), (lm_x - measured_x)) - z.b

    if add_noise:
        noise = np.random.normal(0, 0.005, 3)
        measured_x += noise[0]
        measured_y += noise[1]
        measured_theta += 5*noise[2]

    return PoseStamped(pose.t, measured_x, measured_y, measured_theta)


if __name__ == '__main__':
    print "[WARNING] measure.py should not be run as main"
