#!/usr/bin/env python

# http://robotics.mech.northwestern.edu/~jarvis/hackathon_2016_site/challenge_rrt.html

import numpy as np
import random
import math
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from scipy.misc import imread
from matplotlib import colors
import time


def buffer_world(world):
    # create a new world with inflated obstacle outline
    buffered_world = world.copy()
    for x in range(1, len(world)-1):
        for y in range(1, len(world[0])-1):
            if world[x][y] == 1:
                for u in range(-1, 2):
                    for v in range(-1, 2):
                        buffered_world[x + u][y + v] = 1
    return buffered_world


def Bresenham(start, end):
    """
    Bresenham's Line Algorithm
    Produces a list of tuples from start and end

    points1 = get_line((0, 0), (3, 4))
    points2 = get_line((3, 4), (0, 0))
    assert(set(points1) == set(points2))
    print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points


def endpoint_collision_detect(new_point):
    x = new_point[0]
    y = new_point[1]
    if x <= 1 or x >= grid_size - 2:
        return True
    if y <= 1 or y >= grid_size - 2:
        return True
    if buffered_world[y][x]: # reminder: world coordinates are flipped
        return True
    else:
        return False


def calc_nearest_neighbor(new_point, points):
    min_dist = grid_size * 2
    min_dist_point_index = 0
    for counter, last_point in enumerate(points):
        # dist = math.sqrt(math.pow(abs(new_point[0] - last_point[0]), 2) + math.pow(abs(new_point[1] - last_point[1]), 2))
        dist = calc_dist(new_point, last_point)
        if dist < min_dist:
            min_dist = dist
            min_dist_point_index = counter
    return min_dist_point_index, min_dist


def line_collision_detection(new_point, nearest_point):
    line_pixels = Bresenham(new_point, nearest_point)
    for x, y in line_pixels:
        if buffered_world[y][x]: # reminder: world coordinates are flipped
            return True
    return False


def calc_dist(p1, p2):
    dist = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    return dist


def generate_new_point(points, parents):
    while(True):
        x = np.random.randint(b, grid_size - b + 1)
        y = np.random.randint(b, grid_size - b + 1)
        new_point = [x, y]

        if new_point in points:
            # print "INFO: potential point already exists - point not created"
            continue

        if endpoint_collision_detect(new_point):
            #print "INFO: potential point collides with environment - point not created"
            continue

        nearest_neighbor_index, nearest_neighbor_dist = calc_nearest_neighbor(new_point, points)
        nearest_point = points[nearest_neighbor_index]

        if nearest_neighbor_dist > alpha:
            dx = new_point[0] - nearest_point[0]
            dy = new_point[1] - nearest_point[1]
            new_point[0] = nearest_point[0] + int(dx * alpha / nearest_neighbor_dist)
            new_point[1] = nearest_point[1] + int(dy * alpha / nearest_neighbor_dist)
            # print "INFO: distance greater than allowed - point not created"
            # continue

        if line_collision_detection(new_point, nearest_point):
            #print "INFO: connecting line collides with environment - point not created"
            continue

        parents.append(calc_nearest_neighbor(new_point, points)[0])
        points.append(new_point)
        return new_point


def end_in_sight(new_point):
    # check if there is a clear line to endpoint, if so, path is complete!
    if calc_dist(new_point, q_goal) > alpha:
        return False
    return not line_collision_detection(new_point, q_goal)


def plot_stuff(points, parents):
    # create RRT lines:
    counter = 0
    plot_points = []
    codes = []
    for point in points:
        plot_points.append(points[parents[counter]])
        plot_points.append(point)
        codes.append(Path.MOVETO)
        codes.append(Path.LINETO)
        counter = counter + 1

    # plot the RRT:
    fig = plt.gcf()
    path = Path(plot_points, codes)
    patch = patches.PathPatch(path, color='k')
    ax = fig.add_subplot(111)
    ax.add_patch(patch)
    ax.set_xlim([0, grid_size])
    ax.set_ylim([0, grid_size])

    index = -1 # start at last point of parents data
    route = [q_goal]
    indices = []
    while(index != 0):
        coords = plot_points[2*index]
        route.append(coords)
        indices.append(index)
        index = parents[index]

    route_x = [row[0] for row in route]
    route_y = [row[1] for row in route]
    # fig = plt.gcf()
    plt.scatter(q_init[0], q_init[1], s=150, color='orange', alpha=0.7, label='Start', zorder=99) # plot start point
    plt.scatter(q_goal[0], q_goal[1], s=150, color='green', alpha=1, label='Goal', zorder=99) # plot end point
    plt.plot(route_x, route_y, lw=3, color='orange', alpha=0.7)

    return plot_points



if __name__ == '__main__':
    # initialize variables, grid, and initial elements
    world = imread("N_map.png")
    world = np.flipud(world)
    q_init = [40, 40]
    q_goal = [60, 60]
    max_iterations = 1000
    grid_size = world.shape[0] # assumes square
    alpha = 10 # max distance between new point and nearest current points
    b = 2 # clearance buffer around edges, looks better with one

    # set up 'N' logo in background
    buffered_world = buffer_world(world) # add buffer around 'N'
    cmap = colors.ListedColormap(['white', (78/255., 42/255., 132/255.)]) # white and NU purple
    plt.imshow(world, cmap=cmap, interpolation='nearest', origin='lower', extent=[0, world.shape[0], 0, world.shape[1]])
    plt.axis('off')
    # ax = plt.gca()
    # ax.set_xticks(np.arange(0, 100, 5));
    # ax.set_yticks(np.arange(0, 100, 5));
    # ax.grid()

    # generate new random point (all interference checks done during point generation), then check if finished
    points = [q_init]
    parents = [0]
    i = 1
    while True:

        valid_point = generate_new_point(points, parents)

        if end_in_sight(valid_point):
            parents.append(len(points) - 1)
            points.append(q_goal)
            print "SOLUTION FOUND AFTER", i + 1, "ITERATIONS"

            # time to plot some stuff
            plot_points = plot_stuff(points, parents)
            plt.savefig("figures/output" + str(int(time.time())) + ".png", bbox_inches='tight')
            plt.show()
            break # solution found, we're done

        if i > max_iterations:
            print "MAX # ITERATIONS REACHED (", max_iterations, ")"
            print "NO SOLUTION FOUND"
            break # no solution found in allowed # iterations

        i += 1

### END OF SCRIPT ###
