#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import numpy as np
# import matplotlib as mpl
import matplotlib.pyplot as plt
from datetime import datetime

# my imports:
from definitions import PositionData
import control
from filters import filt_bw
from LWR import LWR
from noisy_sine import noisy_sine
from evaluate_performance import condition_sigma_2_error, calc_xval_error
from convert import deltas_to_positions
from NN_compare_code import NN_compare


def main():
    # load saved data from files - can be remade by running functions in build_dataset.py
    print "loading data from files 'x.dat' and 'u.dat'..."
    t0 = datetime.now()

    x = np.loadtxt('x.dat')
    u = np.loadtxt('u.dat')

    tf = datetime.now()
    print "data loaded in", tf - t0, "s\n"



    ############################################################################
    ########################### CREATE DEADREC DATA ############################
    ############################################################################
    # set the initial position to the ground truth initial position
    initial_position = x[0] # contains initial time, x, y, and theta

    # initialize containers for storing data for plotting later
    deadrec = PositionData(initial_position)
    groundtruth = PositionData(initial_position)
    deltas = PositionData([initial_position[0], 0, 0, 0])

    # initialize required variables
    mu = initial_position
    deadrec_position = initial_position
    position = initial_position
    true_position = initial_position

    # run the deadrec algorithm to get and store the points
    i = 1 # start at 1 since we need a t-1
    for t, v, w in u: # use the contoller model to dead reckon our position
        mu, delta = control.deadrec(u, mu, i, has_noise=False) # get dead rec coordinates
        deadrec.append_data(mu) # collect these points for plotting later
        deltas.append_data([t, delta[0], delta[1], delta[2]]) # collect these points for plotting later
        if i > len(u) - 2:
            break
        i += 1



    ############################################################################
    #### CREATE VARIABLES TO USE FOR ANALYSIS & EXPLORING LEARNING AIM, ETC ####
    ############################################################################
    ### CREATE GROUNDTRUTH DATA VARIABLES
    t_gt = x[:, 0]
    dt_gt = np.append(0.01, np.diff(t_gt)) # append initial dt to beginning, assume it's 0.01

    # dx_gt = np.gradient(x[:, 1])
    # dy_gt = np.gradient(x[:, 2])
    dx_gt = np.diff(x[:, 1])
    dx_gt = np.hstack(([0], dx_gt))
    dy_gt = np.diff(x[:, 2])
    dy_gt = np.hstack(([0], dy_gt))
    dd_gt = np.sqrt(np.add(np.square(dx_gt), np.square(dy_gt)))
    dd_gt = filt_bw(dd_gt, 1) # filter our derived quantity
    # dtheta_gt = np.gradient(x[:, 3])
    dtheta_gt = np.diff(x[:, 3])
    dtheta_gt = np.hstack(([0], dtheta_gt))
    dtheta_gt = np.clip(dtheta_gt, -0.05, 0.05) # clip huge spikes in data, were causing big problems with algorithm outputs (note these are deltas, not absolutes)

    vx_gt = np.divide(dx_gt, dt_gt)
    vy_gt = np.divide(dy_gt, dt_gt)
    v_gt = np.sqrt(np.add(np.square(vx_gt), np.square(vy_gt)))
    v_gt = filt_bw(v_gt, 3)
    ax_gt = np.divide(np.gradient(vx_gt), dt_gt)
    ay_gt = np.divide(np.gradient(vy_gt), dt_gt)
    a_gt = np.sqrt(np.add(np.square(ax_gt), np.square(ay_gt)))
    w_gt = np.divide(dtheta_gt, dt_gt)
    w_gt = [abs(val)/val*2.5 if abs(val) > 2.5 else val for val in w_gt] # huge spikes in data, capping them at 2.5
    alpha_gt = np.divide(np.gradient(w_gt), dt_gt)


    ### CREATE CONTROL DATA VARIABLES
    t_u = u[:, 0]
    dt_u = np.append(0.01, np.diff(t_u)) # need to add a data point at the beginning, assume it's 0.01

    dx_u = deltas.get_dx()
    dy_u = deltas.get_dx()
    dd_u = np.sqrt(np.add(np.square(dx_u), np.square(dy_u)))
    dd_u = filt_bw(dd_u, 1)
    dtheta_u = deltas.get_dtheta()
    v_u = u[:, 1]
    vx_u = np.multiply(v_u, np.cos(deadrec.get_theta()))
    vy_u = np.multiply(v_u, np.sin(deadrec.get_theta()))
    ax_u = np.divide(np.gradient(vx_u), dt_u)
    ay_u = np.divide(np.gradient(vy_u), dt_u)
    a_u = np.sqrt(np.add(np.square(ax_u), np.square(ay_u)))
    a_u = filt_bw(a_u, 1)

    w_u = u[:, 2]
    alpha_u = np.divide(np.gradient(w_u), dt_u)
    alpha_u = filt_bw(alpha_u, 1)


    ### CREATE DELTA AND ERROR VARIABLES
    x_error = np.subtract(x[:, 1], deadrec.get_x())
    x_error = filt_bw(x_error, 1)
    y_error = np.subtract(x[:, 2], deadrec.get_y())
    y_error = filt_bw(y_error, 1)
    theta_error = np.subtract(x[:, 3], deadrec.get_theta())
    theta_error = abs(filt_bw(theta_error, 1))
    d_error = np.sqrt(np.add(np.square(x_error), np.square(y_error)))
    d_error = filt_bw(d_error, 1)

    dx_error = np.subtract(dx_gt, dx_u)
    dx_error = filt_bw(dx_error, 1)
    dy_error = np.subtract(dy_gt, dy_u)
    dy_error = filt_bw(dy_error, 1)
    dd_error = np.subtract(dd_gt, dd_u)
    dtheta_error = np.subtract(dtheta_gt, dtheta_u)
    dtheta_error = filt_bw(dtheta_error, 1)

    v_error = np.subtract(v_gt, v_u)
    v_error = filt_bw(v_error, 1)
    vx_error = np.subtract(vx_gt, vx_u)
    vx_error = filt_bw(vx_error, 1)
    vy_error = np.subtract(vy_gt, vy_u)
    vy_error = filt_bw(vy_error, 1)
    w_error = np.subtract(w_gt, w_u)
    w_error = filt_bw(w_error, 1)
    dw_u = np.diff(w_u)
    dw_u = np.insert(dw_u, 0, 0)



    ############################################################################
    ######################### PLOT DEADRECKONING ERROR #########################
    ############################################################################
    # the goal of the learning algorithm is to reduce or eliminate this error and
    # allow for more accurate navigation, need to show what quantity we're reducing
    plt.figure('Deadreckoning Error', figsize=(15,10))
    plt.plot(range(len(dd_error)), dd_error, label='Position Error')
    plt.plot(range(len(dtheta_error)), dtheta_error, label='Heading Error')
    plt.title("Deadreckoning Error")
    plt.xlim([0, 2000])
    plt.ylim([-0.01, 0.01])
    plt.legend()
    # plt.show()

    plt.figure('Deadreckoning Error - Accumulated', figsize=(15,10))
    plt.plot(range(len(d_error)), d_error, label='Position Error')
    plt.plot(range(len(theta_error)), theta_error, label='Heading Error')
    plt.title("Deadreckoning Error - Accumulated")
    plt.xlim([0, 2000])
    plt.ylim([-0.01, 0.25])
    plt.legend()
    # plt.show()



    ############################################################################
    ########################### NOISY SINE WAVE TEST ###########################
    ############################################################################
    ### GENERATE DATA
    training_factor = 0.30 # percent of data to use for "training"

    print "executing learning algorithm on noisy sine wave..."
    t0 = datetime.now()

    test_x, test_y = noisy_sine(n_cycles=5, n_points=500, noise_factor=1) # generate and get noisy sine_wave data

    training_input = []
    training_output = []
    for n in np.random.randint(0, len(test_x), len(test_x)*training_factor):
        training_input.append(test_x[n])
        training_output.append(test_y[n])

    data_to_learn = test_x

    ### LEARN
    test_lwr = LWR('test') # instantiate algorithm base class with parameters
    test_lwr.set_training_input(training_input) # load training inputs
    test_lwr.set_training_output(training_output) # load training outputs

    results = []
    var2s = []
    for values in data_to_learn:
        output, weights, var2 = test_lwr.predict(values)
        results.append(output)
        var2s.append(var2)

    y_coords = [coord[0] for coord in results]

    print "...done"
    tf = datetime.now()
    print "elapsed time: ", tf-t0, '\n'

    sig2_v, sig2_w = condition_sigma_2_error(var2s, results)
    print "sigma^2 for v & w (percentages):"
    print sig2_v, sig2_w, '\n'


    ### PLOT
    plt.figure('Noisy Sine Test', figsize=(15,10))
    plt.plot(test_x, test_y) # plot original data in background
    plt.scatter(data_to_learn, y_coords, c='red', marker='o') # plot learned data
    plt.title("Tuning Kernel Parameter on Noisy Sinusoid Data")
    plt.axis([0, 10, -1.2, 1.2])
    # plt.show()


    ############################################################################
    ########################## LEARNING COHERENT DATA ##########################
    ############################################################################
    ### GENERATE, LEARN, & PLOT PATH DATA (LEARNING GOAL)
    ### GENERATE DATA
    start1 = 0
    L = 4000 # limit our data to start out, remove after gains are tuned
    p = int(0.25 * L) # number of training data points to sample randomly (without replacement) from the L data points above
    N = 1000 # number of query points to run (first N points of path)

    # create training input data for learning algorithm
    input_vdt = np.multiply(v_u[start1:start1+L].T, dt_u[start1:start1+L])
    input_wdt = np.multiply(w_u[start1:start1+L].T, dt_u[start1:start1+L])
    training_input1 = np.vstack((input_vdt, input_wdt))
    training_input1 = training_input1.T

    # create training output data for learning algorithm
    output_vdt = dd_gt[start1:start1+L].T
    output_wdt = dtheta_gt[start1:start1+L].T
    training_output1 = np.vstack((output_vdt, output_wdt))
    training_output1 = training_output1.T

    # randomly sample newly created training data for appropriate training points
    rand_indx = np.random.choice(L, p, replace=False)
    training_input1 = training_input1[rand_indx, :]
    training_output1 = training_output1[rand_indx, :]
    data_to_learn1 = np.vstack((input_vdt[0:N], input_wdt[0:N]))
    data_to_learn1 = data_to_learn1.T

    # save data to file, per problem instructions
    print "saving training input data..."
    np.savetxt('training_input.dat', training_input1, delimiter=", ")
    print "saved to file: \'training_input.dat\'\n"

    print "saving training output data..."
    np.savetxt('training_output.dat', training_output1, delimiter=", ")
    print "saved to file: \'training_output.dat\'\n"


    ### LEARN DATA
    print "executing learning algorithm on control data..."
    t0 = datetime.now()

    lwr = LWR() # instantiate algorithm base class with parameters
    lwr.set_training_input(training_input1) # load training inputs into algorithm
    lwr.set_training_output(training_output1) # load training outputs into algorithm

    results = []
    var2s = []
    for q in data_to_learn1:
        output, weights, var2 = lwr.predict(q)
        results.append(output) # store output vdt, wdt values
        var2s.append(var2) # store output variance (sigma^2) values

    print "...done"
    tf = datetime.now()
    print "elapsed time: ", tf-t0, '\n'

    sig2_v, sig2_w = condition_sigma_2_error(var2s, results)
    print "sigma^2 for v & w (percentages):"
    print sig2_v, sig2_w

    initial_position1 = x[start1] # contains initial time, x, y, and theta
    x_coords1, y_coords1, theta_coords1 = deltas_to_positions(results, initial_position1, t_u, start1) # convert deltas to X-Y coordinates


    ### PLOT RESULTS
    deadrec_x1 = deadrec.x[start1:start1+N]
    deadrec_y1 = deadrec.y[start1:start1+N]

    plt.figure('Path Comparison - Coherent Data', figsize=(15,10))
    plt.plot(x[start1:start1+N, 1], x[start1:start1+N, 2], c='green', label='Groundtruth') # plot original data in background
    plt.plot(deadrec_x1, deadrec_y1, c='cyan', label='Dead Reckoning') # plot deadreckoning results
    plt.plot(x_coords1, y_coords1, c='red', label='Learned') # plot learned results
    plt.title("Learned, Dead-Reckoned, & Groundtruth Path Data - Coherent Data")
    plt.legend()
    # plt.show()



    ############################################################################
    ######################### LEARNING DISJOINTED DATA #########################
    ############################################################################
    ### USING TRAINING DATA ON DISJOINTED DATA (LEARNING GOAL)
    ### GENERATE DATA
    start2 = 20000

    # create training input data for learning algorithm
    input_vdt = np.multiply(v_u[start2:start2+L].T, dt_u[start2:start2+L])
    input_wdt = np.multiply(w_u[start2:start2+L].T, dt_u[start2:start2+L])
    # training_input2 = np.vstack((input_vdt, input_wdt))
    # training_input2 = training_input2.T

    # create training output data for learning algorithm
    # output_vdt = dd_gt[start2:start2+L].T
    # output_wdt = dtheta_gt[start2:start2+L].T
    # training_output2 = np.vstack((output_vdt, output_wdt))
    # training_output2 = training_output2.T

    # randomly sample newly created training data for appropriate training points
    # rand_indx = np.random.choice(L, p, replace=False)
    # training_input2 = training_input2[rand_indx, :]
    # training_output2 = training_output2[rand_indx, :]
    data_to_learn2 = np.vstack((input_vdt[0:N], input_wdt[0:N]))
    data_to_learn2 = data_to_learn2.T


    ### LEARN DATA
    print "\nexecuting learning algorithm on control data..."
    t0 = datetime.now()

    lwr2 = LWR() # instantiate algorithm base class with parameters
    # lwr2.set_training_input(training_input2) # load training inputs into algorithm
    # lwr2.set_training_output(training_output2) # load training outputs into algorithm
    lwr2.set_training_input(training_input1) # load training inputs into algorithm
    lwr2.set_training_output(training_output1) # load training outputs into algorithm

    results = []
    var2s = []
    for q in data_to_learn2:
        output, weights, var2 = lwr2.predict(q)
        results.append(output) # store output vdt, wdt values
        var2s.append(var2) # store output variance (sigma^2) values

    print "...done"
    tf = datetime.now()
    print "elapsed time: ", tf-t0, '\n'

    sig2_v, sig2_w = condition_sigma_2_error(var2s, results)
    print "sigma^2 for v & w (percentages):"
    print sig2_v, sig2_w

    initial_position2 = x[start2] # contains initial time, x, y, and theta
    x_coords2, y_coords2, theta_coords2 = deltas_to_positions(results, initial_position2, t_u, start2) # convert deltas to X-Y coordinates


    ### CREATE DEADREC DATA 2
    # initialize containers for storing data for plotting later
    deadrec2 = PositionData(initial_position2)
    mu = initial_position2 # initialize required variable

    # update deadrec data, since it has accumulated error from the groundtruth:
    i = start2+1
    for t, v, w in u[start2:start2+L]:
        # use the contoller model to dead reckon our position
        mu, delta = control.deadrec(u, mu, i, has_noise=False) # get dead rec coordinates
        deadrec2.append_data(mu) # collect these points for plotting later
        i += 1

    deadrec_x2 = deadrec2.x[0:N]
    deadrec_y2 = deadrec2.y[0:N]

    ### PLOT RESULTS
    plt.figure('Path Comparison - Disjointed Data', figsize=(15,10))
    plt.plot(x[start2:start2+N, 1], x[start2:start2+N, 2], c='green', label='Groundtruth') # plot original data in background
    plt.plot(deadrec_x2, deadrec_y2, c='cyan', label='Dead Reckoning') # plot deadreckoning results
    plt.plot(x_coords2, y_coords2, c='red', label='Learned') # plot learned results
    plt.title("Learned, Dead-Reckoned, & Groundtruth Path Data - Disjointed Data")
    plt.legend()
    # plt.show()



    ############################################################################
    ########### EVALUATE AND STORE RESULTS FOR CROSS-VALIDATION TEST ###########
    ############################################################################
    ### GENERATE DATA
    start_xval = 0 # anywhere is fine, might as well start at zero
    L = 1000 # limit our data to start out, remove after gains are tuned
    p = L # number of training data points to sample randomly (without replacement) from the L data points above
    N = L # number of query points to run (first N points of path)

    # create training input data for learning algorithm
    input_vdt = np.multiply(v_u[start_xval:start_xval+L].T, dt_u[start_xval:start_xval+L])
    input_wdt = np.multiply(w_u[start_xval:start_xval+L].T, dt_u[start_xval:start_xval+L])
    training_input_xval = np.vstack((input_vdt, input_wdt))
    training_input_xval = training_input_xval.T

    # create training output data for learning algorithm
    output_vdt = dd_gt[start_xval:start_xval+L].T
    output_wdt = dtheta_gt[start_xval:start_xval+L].T
    training_output_xval = np.vstack((output_vdt, output_wdt))
    training_output_xval = training_output_xval.T

    data_to_learn_xval = np.vstack((input_vdt, input_wdt))
    data_to_learn_xval = data_to_learn_xval.T


    ### GATHER CROSS VALIDATION DATA
    print "\ncalculating cross-validation error..."
    t0 = datetime.now()

    xval_error_v, xval_error_w = calc_xval_error(training_input_xval, training_output_xval, data_to_learn_xval)

    print "...done"
    tf = datetime.now()
    print "elapsed time: ", tf-t0, '\n'

    print "cross validation for the first 1000 data points (distance, angle):"
    print xval_error_v, xval_error_w, '\n'

    ### UNCOMMENT THESE LINES TO RUN NEURAL NET COMPARISON ###
    NN_compare(initial_position1, start1, N, x, training_input1, training_output1, data_to_learn1, deadrec_x1, deadrec_y1, x_coords1, y_coords1)
    NN_compare(initial_position2, start2, N, x, training_input1, training_output1, data_to_learn2, deadrec_x2, deadrec_y2, x_coords2, y_coords2)

    plt.show() # show all plots

    print "DONE"
    ### END OF MAIN

if __name__ == '__main__':
    main()

### END OF SCRIPT
