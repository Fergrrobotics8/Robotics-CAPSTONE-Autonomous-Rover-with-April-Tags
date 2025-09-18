#!/usr/bin/python

import numpy as np

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp = 0.5  # Must be positive
        self.ka = 1.5  # Must be positive with ka - kp > 0
        self.kb = -1  # Must be negative
        self.MAX_SPEED = max_speed #0.3
        self.MAX_OMEGA = max_omega #1.6

        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """

        # simplifications
        KP = self.kp
        KA = self.ka
        KB = self.kb

        # derivatives
        delx = goal[0] - state[0]
        dely = goal[1] - state[1]
        theta = state[2]

        p = np.sqrt((delx ** 2) + (dely ** 2))
        alpha = -theta + np.arctan2(dely, delx)
        beta = -theta - alpha

        v = p * KP
        if v > self.MAX_SPEED:
            v = self.MAX_SPEED

        omega = (alpha * KA) + (beta * KB)
        if omega > self.MAX_OMEGA:
            omega = self.MAX_OMEGA

        if p < 0.18:
            done = True
        else:
            done = False

        vw = (v, -omega, done)
        return vw
