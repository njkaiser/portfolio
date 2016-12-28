#!/usr/bin/env python
import numpy as np
import sys
sys.dont_write_bytecode = True

# The robot coordinate system assumes x direction is forward, and
# y direction is left. Therefore a positive linear velocity will add
# to robot x, and a positive angular velocity will add to robot y


def calc_delta(u_t, dt):
    if u_t[2] == 0:
        # special case when the robot is driving in a straight line with
        # no angular velocity
        dx_robot = u_t[1] * dt
        dy_robot = 0.
        dtheta_robot = 0.
    else:
        # for control steps with both forward and angular velocities,
        # we need to calculate trajectory radius
        r = u_t[1] / u_t[2] # this is s = r * theta, rearranged and simplified
        dtheta_robot = u_t[2] * dt
        dx_robot = r * np.sin(dtheta_robot) # robot +x is forward
        dy_robot = r * (1 - np.cos(dtheta_robot)) # robot +y is a left turn

    return [dx_robot, dy_robot, dtheta_robot]


def deadrec(u, position, i, has_noise=True): # IMPLEMENT DEAD RECKONING
    # check there is a control step, otherwise return x_t = x_t-1
    # if u[i][1] < 0.001 and u[i][2] < 0.001:
    #     return position, [0., 0., 0.]
    # SET THINGS UP
    dt = u[i][0] - u[i-1][0]
    d_robot = calc_delta(u[i-1], dt)

    # print "line 37 of control::deadrec position = ", position
    dx = d_robot[0] * np.cos(position[3]) - d_robot[1] * np.sin(position[3])
    dy = d_robot[0] * np.sin(position[3]) + d_robot[1] * np.cos(position[3])
    dtheta = d_robot[2]

    deadrec_x = position[1] + dx # + np.random.normal(readrec_x, 0.05)
    deadrec_y = position[2] + dy
    deadrec_theta = position[3] + dtheta

    # add some random noise to our control step
    if has_noise:
        noise = np.random.normal(0, 0.005, 3)
        deadrec_x += noise[0]
        deadrec_y += 5*noise[1]
        deadrec_theta += 5*noise[2]

    if deadrec_theta > np.pi:
        deadrec_theta -= 2*np.pi
    elif deadrec_theta < -np.pi:
        deadrec_theta += 2*np.pi

    return [u[i][0], deadrec_x, deadrec_y, deadrec_theta], [dx, dy, dtheta]


def main():
    # this does nothing, since we shouldn't be executing this file
    pass


if __name__ == '__main__':
    try:
        main()
    except:
        print "Something went wrong executing control.py"
    finally:
        # put something here if you want it to run for ALL exceptions
        pass
