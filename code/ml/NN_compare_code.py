#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import numpy as np
import matplotlib as mpl
# mpl.rcParams["savefig.directory"] = "/home/njk/Courses/EECS469/HW2/figures"
import matplotlib.pyplot as plt

# must have scikit-learn neural network package installed
from sknn.mlp import Regressor, Layer, Classifier



def NN_compare(initial_position, start, N, x, training_input, training_output, data_to_learn, deadrec_x, deadrec_y, x_coords, y_coords):
    # generate the path data using a Neural Net and compare to the LWR results

    # initialize and instantiate training and test data
    X_train = training_output
    y_train = training_input
    X_to_train = data_to_learn

    # initialize neural network
    nn = Regressor(
    layers=[
        Layer("Rectifier", units=100),
        Layer("Linear")],
    learning_rate=0.02,
    n_iter=10)

    # train the neural network
    nn.fit(X_train, y_train)

    # use freshly-trained neural network to predict output values
    nn_output2 = []
    for i in range(len(X_to_train)):
        X = X_to_train[i]
        X = X.reshape((1,2))
        y = nn.predict(X)
        nn_output2.append(y[0].tolist())

    # transform the deltas into positions for plotting and comparison
    NN_pose = []
    NN_pose.append([initial_position[1], initial_position[2], initial_position[3]])
    NN_x_coords = []
    NN_y_coords = []
    NN_theta_coords = []
    dists2 = []
    angs2 = []
    for i, [dist, ang] in enumerate(nn_output2):
        x_coord = NN_pose[-1][0] + dist * np.cos(NN_pose[-1][2])
        NN_x_coords.append(x_coord)

        y_coord = NN_pose[-1][1] + dist * np.sin(NN_pose[-1][2])
        NN_y_coords.append(y_coord)

        theta_coord = NN_pose[-1][2] + ang
        NN_theta_coords.append(theta_coord)

        NN_pose.append([x_coord, y_coord, theta_coord])
        dists2.append(dist)
        angs2.append(ang)

    # plot the comparison data
    plt.figure(figsize=(15,10))
    plt.plot(x[start:start+N, 1], x[start:start+N, 2], c='green', label='Groundtruth') # plot original data in background
    plt.plot(deadrec_x, deadrec_y, c='cyan', label='Dead Reckoning') # plot deadreckoning results
    plt.plot(x_coords, y_coords, c='red', label='LWR') # plot learned results
    plt.plot(NN_x_coords, NN_y_coords, c='blue', label='Neural Net') # plot learned results
    plt.title("Learned, Dead-Reckoned, & Groundtruth Path Data - Sequential Data")
    plt.legend()
    # plt.show()

    return 0



if __name__ == '__main__':
  pass

### END OF SCRIPT
