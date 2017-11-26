#!/usr/bin/env python

import numpy as np
from definitions import Pose, PoseStamped, MeasurementStamped
from scipy.stats import norm
from params import z_noise, min_prob
# from math import isnan


def calc_expected_measurement(pose, lm):
    '''calculates expected measurement given pose data and a landmark'''

    if isinstance(pose, PoseStamped):
        r_expected = np.sqrt((lm.x - pose.x)**2 + (lm.y - pose.y)**2)
        b_expected = np.arctan2(lm.y - pose.y, lm.x - pose.x) - pose.theta
    elif isinstance(pose, np.ndarray):
        r_expected = np.sqrt((lm.x - pose[:, 0])**2 + (lm.y - pose[:, 1])**2)
        b_expected = np.arctan2(lm.y - pose[:, 1], lm.x - pose[:, 0]) - pose[:, 2]
    else:
        raise Exception('measurement.py::calc_expected_pose() function can only handle inputs of type PoseStamped or numpy ndarray')
    return r_expected, b_expected


# def calc_expected_pose(z, lm):
#     '''calculates expected pose given measurement data and a landmark'''
#
#     theta_expected =
#     x_expected = lm.x - z.r * np.cos(z.theta) +
#
#     return Pose(x_expected, y_expected, theta_expected)


def measurement_model(pose, z, LM, add_noise=False):
    ''' calculates probability of robot pose based given measurement (similar
    to landmark_model_known_correspondence from Probabilistic Robotics)'''

    lm = LM[z.s] # subject (landmark ID)
    r_expected, b_expected = calc_expected_measurement(pose, lm)
    r_prob = norm(r_expected, z_noise.r_rel * r_expected + z_noise.r_abs).pdf(z.r)# + min_prob
    b_prob = norm(b_expected, z_noise.b_abs).pdf(z.b)# + min_prob
    return r_prob * b_prob



if __name__ == '__main__':
    print "[WARNING] measure.py should not be run as main"

    ### UNIT TESTING ###
    import matplotlib.pyplot as plt

    ### RANGE TEST:
    p = PoseStamped(0.0, 0.0, 0.0, 0.0)
    z = MeasurementStamped(0.0, 99, 1.5, 0.0)
    lm = {}
    lm[99] = Pose(3.0, 0.0, 0.0)

    x_axis = []
    prob_graph = []
    for x in np.linspace(0.0, 3.0, 300):
        p.x = x
        prob = measurement_model(p, z, lm)
        x_axis.append(x)
        prob_graph.append(prob)

    print prob_graph
    print len(prob_graph)
    plt.plot(x_axis, prob_graph)
    # plt.plot(x_axis[0:150], prob_graph[0:150])
    # blah = list(reversed(prob_graph[150:]))
    # plt.plot(x_axis[0:150], blah)
    plt.show()
    plt.close()


    ### BEARING TEST:
    p = PoseStamped(0.0, 0.0, 0.0, 0.0)
    z = MeasurementStamped(0.0, 99, 3.0, 0.0)
    lm = {}
    lm[99] = Pose(3.0, 0.0, 0.0)

    x_axis = []
    prob_graph = []
    for theta in np.linspace(-0.78, 0.78, 300):
        p.theta = theta
        prob = measurement_model(p, z, lm)
        x_axis.append(theta)
        prob_graph.append(prob)

    print prob_graph
    print len(prob_graph)
    plt.plot(x_axis, prob_graph)
    # plt.plot(x_axis[0:150], prob_graph[0:150])
    # blah = list(reversed(prob_graph[150:]))
    # plt.plot(x_axis[0:150], blah)
    plt.show()


    ### ARRAY TEST:
    from control import motion_model
    from params import u_noise
    from definitions import ControlStamped

    M = 10 # 10 particles
    chi = np.empty((M, 3))
    chi[:, 0] = np.random.normal(0, u_noise.x_abs, M)
    chi[:, 1] = np.random.normal(0, u_noise.y_abs, M)
    chi[:, 2] = np.random.normal(0, u_noise.theta_abs, M)

    z = MeasurementStamped(0.0, 99, 1.5, 0.0)
    lm = {}
    lm[99] = Pose(3.0, 0.0, 0.0)

    x_final = 3.0
    n_steps = 300
    x_axis = np.linspace(0.0, x_final, n_steps)
    prob_graph = []
    u = ControlStamped(1.0, 0.0, x_axis[1]-x_axis[0], 0.0)
    for x in x_axis:
        prob = measurement_model(chi, z, lm)
        prob_graph.append(prob)
        chi = motion_model(u, chi, add_noise=True)

    # print prob_graph
    # print len(prob_graph)
    fig, ax = plt.subplots()
    for i in xrange(M):
        ax.plot(x_axis, [x[i] for x in prob_graph], label=str(i))

    ax.legend()
    plt.show()
    plt.close()
