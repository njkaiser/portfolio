#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import numpy as np
import matplotlib as mpl
mpl.rcParams["savefig.directory"] = "/home/njk/Courses/EECS469/HW2/figures"
import matplotlib.pyplot as plt

from LWR import LWR
from convert import deltas_to_positions

# from sknn.mlp import Regressor, Layer, Classifier

def condition_sigma_2_error(sigmasquared_vals, input_vals):
    # input raw sigma^2 values from output of algorithm, condition and return

    # condition data: remove huge spikes due to near-zero divisions
    # clip to 1 to eliminate the huge (10^16) spikes (note: 1 is still extremely high)
    sig2_v = [h[0] if h[0] < 1 else 1 for h in sigmasquared_vals]
    sig2_w = [h[1] if h[1] < 1 else 1 for h in sigmasquared_vals]

    # calculate mean and standard deviation to determine outliers
    mean_v = np.mean(np.fabs(sig2_v))
    mean_w = np.mean(np.fabs(sig2_w))
    stdev_v = np.std(sig2_v)
    stdev_w = np.std(sig2_w)

    # then clip again to 6 sigma levels
    clip_v = mean_v + 3 * stdev_v
    clip_w = mean_w + 3 * stdev_w
    sig2_v = [h if h < clip_v else clip_v for h in sig2_v]
    sig2_w = [h if h < clip_w else clip_w for h in sig2_w]

    # now normalize (convert to a percentage) by dividing by the associated step value
    sig2_v_norm = np.divide(sig2_v, [row[0] if row[0] != 0 else mean_v for row in input_vals])
    sig2_w_norm = np.divide(sig2_w, [row[1] if row[1] != 0 else mean_w for row in input_vals])

    # repeat outlier step since we potentially divided by really small numbers
    # calculate mean and standard deviation to determine outliers
    mean_v = np.mean(np.fabs(sig2_v_norm))
    mean_w = np.mean(np.fabs(sig2_w_norm))
    stdev_v = np.std(sig2_v_norm)
    stdev_w = np.std(sig2_w_norm)

    # then clip again to 6 sigma levels
    clip_v = mean_v + 3 * stdev_v
    clip_w = mean_w + 3 * stdev_w
    sig2_v_norm = [h if h < clip_v else clip_v for h in sig2_v_norm]
    sig2_w_norm = [h if h < clip_w else clip_w for h in sig2_w_norm]

    # finally arrive at the output:
    sig2_v_mean = np.mean(sig2_v_norm)
    sig2_w_mean = np.mean(sig2_w_norm)

    ### development only - plotting to ensure outliers are dealt with correctly:
    plt.figure('Sigma^2 Error for v and w', figsize=(15,10))
    plt.plot(range(len(sig2_v_norm)), sig2_v_norm, label='sigma^2 error (v)')
    plt.plot(range(len(sig2_w_norm)), sig2_w_norm, label='sigma^2 error (w)')
    plt.title('Sigma^2 Error for v and w, Wine Wave & Robot Data (Coherent & Disjointed) Runs Aggregated ')
    plt.legend()
    # plt.show()

    return sig2_v_mean, sig2_w_mean



def calc_xval_error(training_input, training_output, data_to_learn):
    ### CALCULATE THE CROSS VALIDATION ERROR
    ### SEE LOCALLY WEIGHTED LEARNING [ATKESON], SECTION 9.3

    lwr_xval = LWR() # instantiate algorithm base class with parameters
    lwr_xval.set_training_input(training_input) # load training inputs into algorithm
    lwr_xval.set_training_output(training_output) # load training outputs into algorithm

    results = []
    var2s = []
    xvals = []
    for q in data_to_learn:
        xval, _, __ = lwr_xval.xvalidate(q) # calculate cross-validation error
        xvals.append(xval) # store cross-validation error values


    # subtract expected value from calculated value
    xvals = [row[0:1] for row in xvals]
    xvals = np.subtract(xvals, training_output)
    xvals = np.square(xvals)
    # print xvals

    # calculate error as a percentage of intended step
    xval_error_v = [row[0] for row in xvals]
    xval_error_w = [row[1] for row in xvals]

    v_vals = [row[0] if row[0] != 0 else 1 for row in training_output]
    w_vals = [row[1] if row[1] != 0 else 1 for row in training_output]

    # eliminate obvious outliers
    xval_error_v = [h if h < 5 else 5 for h in xval_error_v]
    xval_error_w = [h if h < 5 else 5 for h in xval_error_w]

    # calculate mean and standard deviation to determine outliers
    mean_v = np.mean(xval_error_v)
    mean_w = np.mean(xval_error_w)
    stdev_v = np.std(xval_error_v)
    stdev_w = np.std(xval_error_w)

    # then clip again to 6 sigma levels
    clip_v = mean_v + 3 * stdev_v
    clip_w = mean_w + 3 * stdev_w
    xval_error_v = [h if h < clip_v else clip_v for h in xval_error_v]
    xval_error_w = [h if h < clip_w else clip_w for h in xval_error_w]

    # finally arrive at the output:
    xval_error_mean_v = np.mean(np.fabs(xval_error_v))
    xval_error_mean_w = np.mean(np.fabs(xval_error_w))

    # development purposes only:
    plt.figure('Cross-Validation Error for v and w', figsize=(15,10))
    plt.plot(range(len(xval_error_v)), xval_error_v, label='v')
    plt.plot(range(len(xval_error_w)), xval_error_w, label='w')
    plt.title('Cross-Validation Error for v and w')
    plt.legend()
    # plt.show()

    return xval_error_mean_v, xval_error_mean_w


# def NN_compare():
#     # generate the path data using a Neural Net and compare to the LWR results
#     X_train = np.loadtxt('compare_1_output.dat', delimiter=', ')
#     y_train = np.loadtxt('compare_1_input.dat', delimiter=', ')
#
#     X_to_train = np.loadtxt('data_to_learn.dat', delimiter=', ')
#
#     nn = Regressor(
#     layers=[
#         Layer("Rectifier", units=100),
#         Layer("Linear")],
#     learning_rate=0.02,
#     n_iter=10)
#
#     nn.fit(X_train, y_train)
#
#     nn_output2 = []
#     for i in range(len(X_to_train)):
#         X = X_to_train[i]
#         X = X.reshape((1,2))
#         # print X
#         # print np.shape(X)
#         y = nn.predict(X)
#         nn_output2.append(y[0].tolist())
#         # print y[0]
#
#     # y_example = nn.predict(X_example)
#     # print nn_output2
#     # np.savetxt('nn_output2_1.dat', nn_output2, delimiter=", ")
#     # compare1 = np.loadtxt('compare1.dat', delimiter=', ')
#
#     initial_position = [1.29812900e+00, 1.88315210e+00, 2.82870000e+00]
#     pose2 = []
#     pose2.append(initial_position)
#     x_coords2 = []
#     y_coords2 = []
#     theta_coords2 = []
#     dists2 = []
#     angs2 = []
#     for i, [dist, ang] in enumerate(nn_output2):
#         x_coord = pose2[-1][0] + dist * np.cos(pose2[-1][2])
#         x_coords2.append(x_coord)
#
#         y_coord = pose2[-1][1] + dist * np.sin(pose2[-1][2])
#         y_coords2.append(y_coord)
#
#         theta_coord = pose2[-1][2] + ang
#         theta_coords2.append(theta_coord)
#
#         pose2.append([x_coord, y_coord, theta_coord])
#         dists2.append(dist)
#         angs2.append(ang)
#
#
#
#     plt.figure('path tracking', figsize=(15,10))
#     # plt.plot(x[0:N, 1], x[0:N, 2], c='green', label='groundtruth') # plot original data in background
#     # plt.plot(deadrec.x[0:N], deadrec.y[0:N], c='cyan', label='dead reckoning') # plot deadreckoning results
#     # plt.plot(x_coords1, y_coords1, c='black', label='learned1') # plot learned results
#     plt.plot(x_coords2, y_coords2, c='blue', label='learned2') # plot learned results
#     plt.title("Learned, Dead-Reckoned, & Groundtruth Path Data")
#     plt.legend()
#     # plt.axis([0, 10, -1.2, 1.2])
#     plt.show()
#
#     # plt.plot([row[0] for row in nn_output2], [row[1] for row in nn_output2])
#     # plt.show()




if __name__ == '__main__':
  pass

### END OF SCRIPT
