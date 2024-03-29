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
            # get Predicted endpoint postion
            predicted_xy = arm.get_endpoint(angle)
            # calculate root squared error
            error = np.sqrt((np.array(x_y) - np.array(predicted_xy))**2)

            # print errors that exceed the threshold
            if np.sum(error) > error_threshold:
                print('********* Trail #{0:d} *********'.format(trail_counter+1))
                print('Task: ')
                print('  Initial joint angles', arm.joint_angle)
                print('Result: ')
                print('  Reached joint angles: ', angle)
                print('Cost: ')
                print('  Target endpoint position: ', x_y)
                print('  Predicted endpoint position: ', predicted_xy)
                print('Summary: ')
                print('  Error: ', error)
                print('Running time: ', (time.time() - start_time)*1000, "ms")
                print('------------------------------')
            trail_counter += 1

# 
# Main: Testing
# 
x = np.arange(150, 170, 5)
y = np.arange(60, 90, 5)
threshold = 0.0

evaluate(x, y, threshold)