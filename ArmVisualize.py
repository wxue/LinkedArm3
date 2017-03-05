#!/usr/bin/python3
# -*- coding: utf-8 -*-

import pyglet
import numpy as np

import ArmMC

def visualize():
    arm = ArmMC.LinkedArm3()

    # open a visualized window
    window = pyglet.window.Window()

    label = pyglet.text.Label('<3-Link-Arm> Initial Endpoint',
        font_size=12, x=window.width//2, y=window.height//2,
        anchor_x='center', anchor_y='center')

    def get_jointpoints():
        x = np.array([ 0, 
            arm.arm_length[0]*np.cos(arm.joint_angle[0]),
            arm.arm_length[0]*np.cos(arm.joint_angle[0]) + arm.arm_length[1]*np.cos(arm.joint_angle[0]+arm.joint_angle[1]),
            arm.arm_length[0]*np.cos(arm.joint_angle[0]) + arm.arm_length[1]*np.cos(arm.joint_angle[0]+arm.joint_angle[1]) + 
                arm.arm_length[2]*np.cos(np.sum(arm.joint_angle)) ]) + window.width/2

        y = np.array([ 0, 
            arm.arm_length[0]*np.sin(arm.joint_angle[0]),
            arm.arm_length[0]*np.sin(arm.joint_angle[0]) + arm.arm_length[1]*np.sin(arm.joint_angle[0]+arm.joint_angle[1]),
            arm.arm_length[0]*np.sin(arm.joint_angle[0]) + arm.arm_length[1]*np.sin(arm.joint_angle[0]+arm.joint_angle[1]) + 
                arm.arm_length[2]*np.sin(np.sum(arm.joint_angle)) ])

        return np.array([x, y]).astype('int')
    
    window.jps = get_jointpoints()

    @window.event
    def on_draw():
        window.clear()
        label.draw()
        for i in range(3): 
            pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', 
                (window.jps[0][i], window.jps[1][i], 
                 window.jps[0][i+1], window.jps[1][i+1])), ('c3B', (0, 0, 255, 0, 255, 0)))

    @window.event
    def on_mouse_motion(x, y, dx, dy):
        x_y = [x, y]
        angle = arm.inverse_kinematics([x - window.width/2, y])
        arm.joint_angle = angle
        # get actual endpoint postion
        actual_xy = arm.get_endpoint(angle)
        # calculate root squared error
        error = np.sqrt((np.array(x_y) - np.array(actual_xy))**2)

        label.text = 'Target Point: (%.3f, %.3f) Angels: (%.3f, %.3f, %.3f)'%(x,y,angle[0],angle[1],angle[2])
        window.jps = get_jointpoints()

    pyglet.app.run()

# 
# Main: Visualization
# 
visualize()
