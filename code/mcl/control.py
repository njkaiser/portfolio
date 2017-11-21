#!/usr/bin/env python

import numpy as np
from definitions import PoseStamped
from params import u_noise

# The robot coordinate system assumes x direction is forward, and
# y direction is left. A positive linear velocity will therefore add
# to robot x, and a positive angular velocity will add to robot y


def calc_delta(u):
    if u.w == 0:
        # case: robot is driving straight (no angular velocity)
        robot_dx = u.v * u.dt
        robot_dy = 0.0
        robot_dtheta = 0.0
    else:
        # case: robot driving in arc (calculate trajectory radius)
        r = u.v / u.w # this is s = r * theta, rearranged and simplified
        robot_dtheta = u.w * u.dt
        robot_dx = r * np.sin(robot_dtheta) # robot +x is forward
        robot_dy = r * (1 - np.cos(robot_dtheta)) # robot +y is a left turn

    return robot_dx, robot_dy, robot_dtheta


def motion_model(u, pose, add_noise=False):
    '''dead reckon current pose given control input, previous pose, and dt'''

    # TODO: this may not be necessary when filter is implemented, revisit
    # check if there's a control input, otherwise return x_t = x_t-1 (prevents drift when robot isn't moving)
    if abs(u.v) < 0.001 and abs(u.w) < 0.001:
        return pose

    # calculate delta pose in robot coordinates
    robot_dx, robot_dy, robot_dtheta = calc_delta(u)


    if isinstance(pose, PoseStamped):
        # transform robot deltas to global coordinate frame
        dx = robot_dx * np.cos(pose.theta) - robot_dy * np.sin(pose.theta)
        dy = robot_dx * np.sin(pose.theta) + robot_dy * np.cos(pose.theta)
        dtheta = robot_dtheta

        # add noise to our control step
        if add_noise:
            dx *= np.random.normal(1, u_noise.x_rel, 1)
            dy *= np.random.normal(1, u_noise.y_rel, 1)
            dtheta = dtheta * np.random.normal(1, u_noise.theta_rel, 1) + np.random.normal(0, u_noise.theta_abs, 1)

        # calculate global pose from deltas
        pose_new = PoseStamped(pose.t + u.dt, pose.x + dx, pose.y + dy, pose.theta + dtheta)

        return pose_new

    elif isinstance(pose, np.ndarray):

        if add_noise:
            # random noise vectors for x, y, theta
            n_dx = np.random.normal(1, u_noise.x_rel, len(pose))
            n_dy = np.random.normal(1, u_noise.y_rel, len(pose))
            n_dtheta_rel = np.random.normal(1, u_noise.theta_rel, len(pose))
            n_dtheta_abs = np.random.normal(0, u_noise.theta_abs, len(pose))

            # transform robot deltas to global coordinate frame (plus noise)
            dx = n_dx * robot_dx * np.cos(pose[:, 2] + n_dtheta_abs) - n_dy * robot_dy * np.sin(pose[:, 2] + n_dtheta_abs)
            dy = n_dx * robot_dx * np.sin(pose[:, 2] + n_dtheta_abs) + n_dy * robot_dy * np.cos(pose[:, 2] + n_dtheta_abs)
            dtheta = n_dtheta_rel * robot_dtheta + n_dtheta_abs

        else:
            # transform robot deltas to global coordinate frame
            dx = robot_dx * np.cos(pose[:, 2]) - robot_dy * np.sin(pose[:, 2])
            dy = robot_dx * np.sin(pose[:, 2]) + robot_dy * np.cos(pose[:, 2])
            dtheta = robot_dtheta

        # calculate global pose from deltas
        pose_new = np.column_stack((pose[:, 0] + dx, pose[:, 1] + dy, pose[:, 2] + dtheta))

        return pose_new

    else:
        raise Exception('control.py::motion_model() function can only handle inputs of type PoseStamped or numpy ndarray')


if __name__ == '__main__':
    print "[WARNING] control.py should not be run as main"
