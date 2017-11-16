#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from particle_filter import particle_filter

def PathTrace(data, plotname, hold, color, label):
    plt.figure(1)
    plt.subplot(111)
    plt.plot([a.x for a in data], [a.y for a in data], color=color, label=label)
    if not hold:
        plt.title(plotname)
        plt.xlabel('x position [m]')
        plt.ylabel('y position [m]')
        plt.legend()
        plt.show()


def plot_particles(PF):#fig, ax, PF):
    '''plot particles (represented by arrows)'''
    # fig, ax = fig, ax
    fig, ax = plt.subplots()

    mag = 0.01 # in meters
    theta = np.array([row[2] for row in PF.chi])
    X = np.array([row[0] for row in PF.chi])
    Y = np.array([row[1] for row in PF.chi])
    U = X + mag*np.cos(theta)
    V = Y + mag*np.sin(theta)

    C = np.linspace(0, 255, len(X))
    QV = ax.quiver(X, Y, U, V, C)#, angles='xy', scale_units='xy', width=0.003, headwidth=2, scale=1, color='grey', label='Particles')
    # QV = ax.quiver(X, Y, U, V, angles='xy', scale_units='xy', width=0.003, headwidth=2, scale=1, color='grey', label='Particles')
    # plt.quiverkey(QV, 1.2, 0.65, 1, 'arrow 1', coordinates='axes')

    # plt.axis('equal')
    # plt.axis([-0.1, 1.6, -1.6, 0.1])
    # ax.set_xlim(-0.1, 1.6)
    # ax.set_ylim(-0.3, 0.1)
    plt.xlim(-0.1, 1.6-0.15)
    plt.ylim(-0.3-0.35, 0.1)
    plt.gca().set_aspect('equal', adjustable='box')

    plt.draw()
    plt.show()

    return fig, ax


if __name__ == '__main__':
    print "[WARNING] plot.py should not be run as main"
