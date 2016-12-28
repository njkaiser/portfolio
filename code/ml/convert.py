#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import numpy as np


def deltas_to_positions(results, initial_position, t, start):
    # convert deltas to X-Y coordinates
    pose = []
    pose.append(initial_position)
    x_coords = []
    y_coords = []
    theta_coords = []
    dists = []
    angs = []
    for i, [dist, ang, onescolumn] in enumerate(results):
        x_coord = pose[-1][1] + dist * np.cos(pose[-1][3])
        x_coords.append(x_coord)

        y_coord = pose[-1][2] + dist * np.sin(pose[-1][3])
        y_coords.append(y_coord)

        theta_coord = pose[-1][3] + ang
        theta_coords.append(theta_coord)

        pose.append([t[start + i], x_coord, y_coord, theta_coord])
        dists.append(dist)
        angs.append(ang)

    return x_coords, y_coords, theta_coords



if __name__ == '__main__':
  pass

### END OF SCRIPT
