#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import numpy as np


def noisy_sine(n_cycles = 5, n_points = 500, noise_factor=1):
    # parameters for sine wave:
    a = 1 # amplitude
    T = 2. # period
    x_var = 0.005 * noise_factor # variance of noise in x (maybe make this 0, otherwise too noisy?)
    y_var = 0.025 * noise_factor # variance of noise in y

    # generate sine wave points:
    x = []
    y = []
    for t in np.linspace(0, n_cycles * T, n_points):
        # x_coord = i + x_var * np.random.randn(1)
        x_coord = t
        x.append(x_coord)
        y_coord = np.sin(2 * np.pi / T * t) + y_var * np.random.randn(1)
        y.append(y_coord[0])

    return x, y
    ### END noisy_sine()


if __name__ == '__main__':
    noisy_sine()

### END OF SCRIPT
