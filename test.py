#!/usr/bin/python3
# -*- coding: utf-8 -*-

import time
import math
import numpy as np
import scipy.optimize

import ArmMC

def evaluate(range_of_x, range_of_y, error_threshold):
    arm = ArmMC.LinkedArm3()

    # initialize
    trail_counter = 0
    start_time = time.time()

    for ith_x in range(len(range_of_x)):
        for ith_y in range(len(range_of_y)):
            x_y = [range_of_x[ith_x], range_of_y[ith_y]]
            # [x,y] -> angle
            angle = arm.inverse_kinematics(xy=x_y)
            # get actual endpoint postion
            actual_xy = arm.get_endpoint(angle)
            # calculate root squared error
            error = np.sqrt((np.array(x_y) - np.array(actual_xy))**2)

            # print errors that exceed the threshold
            if np.sum(error) > error_threshold:
                print('********* Trail #{0:d} *********'.format(trail_counter))
                print('Task: ')
                print('  Initial joint angles', arm.joint_angle)
                print('  Reached joint angles: ', angle)
                print('Result: ')
                print('  Correct endpoint position: ', x_y)
                print('  Actual endpoint position: ', actual_xy)
                print('Summary: ')
                print('  Error: ', error)
                print('Running time: ', (time.time() - start_time)*1000, "ms")
                print('------------------------------')
            trail_counter += 1

# 
# Main: Testing
# 
x = np.arange(0, 5, 0.5)
y = np.arange(0, 5, 0.5)
threshold = 0.8

evaluate(x, y, threshold)