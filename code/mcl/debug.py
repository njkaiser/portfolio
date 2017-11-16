#!/usr/bin/env python
import numpy as np
import sys
sys.dont_write_bytecode = True

def determine_sensor_model(true_position, z_t, lm):
    # determine our position based on measurement data

    s = z_t[1] # subject number
    r = z_t[2] # range (distance from robot to landmark)
    b = z_t[3] # bearing (angle between robot forward and landmark)
    theta = true_position[3] # need to know which direction we think the robot is pointing

    s_index = None
    for count, row in enumerate(lm):
        if row[0] == s:
            s_index = count
            break
    if s_index is None or s_index in (5, 14, 41, 32, 23):
        # raise LookupError("no landmark")
        return

    lm_x = lm[s_index][1]
    lm_y = lm[s_index][2]

    measured_x = lm_x - r * np.cos(theta + b)
    measured_y = lm_y - r * np.sin(theta + b)
    measured_theta = np.arctan2((lm_y - measured_y), (lm_x - measured_x)) - b

    real_r = np.sqrt((lm_x - true_position[1])**2 + (lm_y - true_position[2])**2)
    real_b = -theta + np.arctan2((lm_y - true_position[2]), (lm_x - true_position[1]))

    r_error = r/real_r
    b_error = b/real_b

    # print "LM#", lm[s_index][0], "\trobot is at ", measured_x, ',', measured_y, ',', measured_theta
    return [true_position[0], measured_x, measured_y, measured_theta], r, b, real_r, real_b


def determine_sensor_error(measured_position_t, true_position_t, z_t, lm):

    r = z_t[2]
    b = z_t[3]
    theta = true_position[3]

    s_index = None
    for count, row in enumerate(lm):
        if row[0] == s:
            s_index = count
            break
    if s_index is None or s_index in (5, 14, 41, 32, 23):
        # raise LookupError("no landmark")
        return

    lm_x = lm[s_index][1]
    lm_y = lm[s_index][2]

    # 7, 9, 18, 25
    # print z_t[1]
    pass




def main():
    # this does nothing, since we shouldn't be executing this file
    pass


if __name__ == '__main__':
    try:
        main()
    except:
        print "Something went wrong executing debug.py"
    finally:
        # put something here if you want it to run for ALL exceptions
        pass
