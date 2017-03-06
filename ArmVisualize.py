#!/usr/bin/python3
# -*- coding: utf-8 -*-

import pyglet
import numpy as np

import ArmMC

def visualize():
    arm = ArmMC.LinkedArm3()

    # open a visualized window
    window = pyglet.window.Window(800, 800)

    half_window_width = window.width//2
    half_window_height = window.height//2

    label = pyglet.text.Label('<3-Link-Arm> Initial Endpoint',
        font_size=12, x=half_window_width, y=half_window_height,
        anchor_x='center', anchor_y='center')

    def get_jointpoints():
        x = np.array([ 0 + half_window_width, 
            arm.arm_length[0]*np.cos(arm.joint_angle[0]) + half_window_width,
            arm.arm_length[0]*np.cos(arm.joint_angle[0]) + arm.arm_length[1]*np.cos(arm.joint_angle[0]+arm.joint_angle[1]) + half_window_width,
            arm.arm_length[0]*np.cos(arm.joint_angle[0]) + arm.arm_length[1]*np.cos(arm.joint_angle[0]+arm.joint_angle[1]) +
                arm.arm_length[2]*np.cos(np.sum(arm.joint_angle))+half_window_width ])

        y = np.array([ 0 + half_window_height, 
            arm.arm_length[0]*np.sin(arm.joint_angle[0])+half_window_height,
            arm.arm_length[0]*np.sin(arm.joint_angle[0]) + arm.arm_length[1]*np.sin(arm.joint_angle[0]+arm.joint_angle[1])+half_window_height,
            arm.arm_length[0]*np.sin(arm.joint_angle[0]) + arm.arm_length[1]*np.sin(arm.joint_angle[0]+arm.joint_angle[1])+ 
                arm.arm_length[2]*np.sin(np.sum(arm.joint_angle)) + half_window_height ])

        return np.array([x, y]).astype('int')
    
    window.arm_segments = get_jointpoints()

    @window.event
    def on_draw():
        window.clear()
        label.draw()
        for i in range(3): 
            pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', 
                (window.arm_segments[0][i], window.arm_segments[1][i], 
                 window.arm_segments[0][i+1], window.arm_segments[1][i+1])), ('c3B', (0, 0, 255, 0, 255, 0)))

    @window.event
    def on_mouse_motion(x, y, dx, dy):
        # locate point[0,0] in the middle
        x -= half_window_width
        y -= half_window_height

        x_y = [x, y]
        angle = arm.inverse_kinematics(xy=x_y)
        # get predicted endpoint postion
        predicted_xy = arm.get_endpoint(angle)
        arm.joint_angle = angle
        # calculate root squared error
        error = np.sqrt((np.array(x_y) - np.array(predicted_xy))**2)

        # monitoring output-- label settings
        label.width = half_window_width*1.4
        label.multiline = True
        label.y = half_window_height/2
        label.text = 'Target Point: (%.14f, %.14f) \
                      Predicted Point: (%.14f, %.14f) \
                      Angels: (%.14f, %.14f, %.14f) \
                      Error: (%.14f, %.14f)'%(x,y,predicted_xy[0],predicted_xy[1],angle[0],angle[1],angle[2],error[0],error[1])
        window.arm_segments = get_jointpoints()
        # print('window.arm_segments: ', window.arm_segments)

    pyglet.app.run()

# 
# Main: Visualization
# 
visualize()
