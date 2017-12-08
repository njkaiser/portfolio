#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from particle_filter import particle_filter


def PathTrace(data, plotname, hold, color, label):
    # plt.figure(1)
    # plt.subplot(111)
    plt.plot([a.x for a in data], [a.y for a in data], color=color, label=label)
    if not hold:
        plt.title(plotname)
        plt.xlabel('x position [m]')
        plt.ylabel('y position [m]')
        plt.legend()
        plt.show()


def plot_particles(fig, ax, particles, weights=0):
    '''plot particles (represented by arrows)'''
    fig, ax = fig, ax # TODO do I need this?

    theta = np.array([row[2] for row in particles])
    X = np.array([row[0] for row in particles])
    Y = np.array([row[1] for row in particles])
    U = np.cos(theta)
    V = np.sin(theta)
    # print "sizes:", X.shape, Y.shape, U.shape, V.shape

    # quiver plot scaling is stupid in matplotlib, need to normalize:
    U = U / np.sqrt(U**2 + V**2);
    V = V / np.sqrt(U**2 + V**2);

    # if isinstance(weights, np.ndarray):
    #     C = weights # we'll use a Yellow-Green-Blue colormap to represent particle weights
    # else:
    #     C = np.linspace(0, 255, len(X)) # taste the rainbow
    C = np.linspace(0, 255, len(X)) # taste the rainbow

    # S = ax.scatter(X, Y, c=C, marker='.')
    Q = ax.quiver(X, Y, U, V, C, angles='xy', scale_units='xy', scale=20, width=0.001, headwidth=4, label='Particles')
    # Q = ax.quiver(X, Y, U, V, C, cmap='YlGnBu', angles='xy', scale_units='xy', scale=20, width=0.001, headwidth=4, label='Particles')

    # plt.axis('equal')
    # plt.axis([-0.1, 1.6, -1.6, 0.1])
    # ax.set_xlim(-0.1, 1.6)
    # ax.set_ylim(-0.3, 0.1)
    # plt.xlim(-0.1, 1.6-0.15)
    # plt.ylim(-0.3-0.35, 0.1)
    plt.gca().set_aspect('equal', adjustable='box')

    plt.draw()
    mng = plt.get_current_fig_manager()
    mng.full_screen_toggle()
    plt.show()

    return fig, ax


if __name__ == '__main__':
    print "[WARNING] plot.py should not be run as main"
