#!/usr/bin/python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import itertools
import scipy.optimize

class LinkedArm3:

    def __init__(self, arm_length=None, joint_angle=None, joint_angle_default=None):
        super(LinkedArm3, self).__init__()
        """
        Basic Configuration
        """
        # set default arm lengths: L1, L2, L3
        self.arm_length = np.array([100, 100, 100]) if arm_length is None else arm_length
        # set default joint angles: Theta1-shoulder, Theta2-elbow, Theta3-wrist
        self.joint_angle_default = np.array([math.pi/6, math.pi/6, math.pi/6]) if joint_angle_default is None else joint_angle_default
        # set angles rotation range
        # self.max_angles = [math.pi*2, math.pi*2, math.pi*2]
        self.max_angles = [math.pi, math.pi/2, math.pi/2]
        self.min_angles = [0, 0, 0]
        
        # initialize joint angles: Theta1, Theta2, Theta3
        self.joint_angle = [math.pi/6, math.pi/6, math.pi/6] if joint_angle is None else joint_angle

    def get_endpoint(self, joint_angle=None):
        """
        Get Current Endpoint:
        Return endpoint location as [x,y]
        """
        if joint_angle is None:
            joint_angle = self.joint_angle

        x = self.arm_length[0]*np.cos(joint_angle[0]) + \
            self.arm_length[1]*np.cos(joint_angle[0]+joint_angle[1]) + \
            self.arm_length[2]*np.cos(np.sum(joint_angle))

        y = self.arm_length[0]*np.sin(joint_angle[0]) + \
            self.arm_length[1]*np.sin(joint_angle[0]+joint_angle[1]) + \
            self.arm_length[2]*np.sin(np.sum(joint_angle))

        return [x, y]

    def inverse_kinematics(self, xy):
        """
        Inverse kinematics algorithm: Using SciPy optimize package minimization function.

        Input: Endpoint location -- [x,y]
        Output: Angels -- [Theta1, Theta2, Theta3]
        """

        def distance_to_default(joint_angle, *args):
            """
            Distance through joint space to the default arm configuration. 

            Input: joint angles -- [Theta1, Theta2, Theta3]
            Output : euclidean distance to the default arm position
            """
            # Weight: each joint moving priority
            weight = [1, 1.5, 2]
            # weight = [1, 1, 1]
            return np.sqrt(np.sum([(ith_joint_angle - ith_joint_angle_default)**2 * ith_weight
                for ith_joint_angle, ith_joint_angle_default, ith_weight in itertools.zip_longest(joint_angle, self.joint_angle_default, weight)]))

        def x_constraint(joint_angle, xy):
            """
            Input: [Theta1, Theta2, Theta3]
            Output: difference between current and target endopint position -- x
            """
            x = (self.arm_length[0]*np.cos(joint_angle[0]) + self.arm_length[1]*np.cos(joint_angle[0]+joint_angle[1]) +
                 self.arm_length[2]*np.cos(np.sum(joint_angle))) - xy[0]
            return x

        def y_constraint(joint_angle, xy):
            """
            Input: [Theta1, Theta2, Theta3]
            Output: difference between current and target endopint position -- y
            """
            y = (self.arm_length[0]*np.sin(joint_angle[0]) + self.arm_length[1]*np.sin(joint_angle[0]+joint_angle[1]) +
                 self.arm_length[2]*np.sin(np.sum(joint_angle))) - xy[1]
            return y

        def raise_constraint(joint_angle, xy):
            """
            Used in the function minimization such that the output from
            this function must be greater than 0 to be successfully passed.
            """
            return self.max_angles - joint_angle

        def fall_constraint(joint_angle, xy):
            """
            Used in the function minimization such that the output from
            this function must be greater than 0 to be successfully passed.
            """
            return joint_angle - self.min_angles

        return scipy.optimize.fmin_slsqp(
            func=distance_to_default,
            x0=self.joint_angle,    # Initial guess for the independent variable(s).
            eqcons=[x_constraint,   # A list of functions of length n such that eqcons[j](x,*args) == 0.0 in a successfully optimized problem.
                    y_constraint],
            ieqcons=[raise_constraint,  # A list of functions of length n such that ieqcons[j](x,*args) >= 0.0 in a successfully optimized problem.
                     fall_constraint],
            args=(xy,),
            iprint=0)   # iprint <= 0 : Silent operation; iprint == 1 : Print summary upon completion (default); iprint >= 2 : Print status of each iterate and summary
