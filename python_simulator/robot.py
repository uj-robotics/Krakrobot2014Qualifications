from defines import *

from math import (
  pi, sqrt, hypot, sin, cos, tan, asin, acos, atan, atan2, radians, degrees,
  floor, ceil, exp
)
import numpy as np
import random
from utils import *

class Robot:
    """ The main class representing robot that can sense and move """

    def __init__(self, length = 0.5):
        """
        Initialize robot
        """

        self.x = 0
        .0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise    = 0.0
        self.distance_noise    = 0.0
        self.sonar_noise = 0.0
        self.measurement_noise = 0.0
        self.num_collisions    = 0
        self.num_steps         = 0
        self.time_limit = 0.0

    #TODO: extract
    def set(self, new_x, new_y, new_orientation):
        """
        Set robot position
        @note: Cannot be called by contestant
        """

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * pi)


    #TODO: extract from this class
    def set_noise(self, new_s_noise, new_d_noise, new_m_noise, new_sonar_noise):
        """
        Set noise parameter
        @note: Cannot be called by contestant
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise     = float(new_s_noise)
        self.distance_noise    = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)
        self.sonar_noise = float(new_sonar_noise)

    #TODO: extract from this class
    def check_collision(self, grid):
        """
        Checks for collisions
        @note: Cannot be called by contestant
        @returns: True if no collisions
        """
        print "Checking collisions ",self.x, " ",self.y," ", self.orientation
        # Box based (sharp edges):
        for i in xrange(len(grid)):
            for j in xrange(len(grid[0])):
                # not sure about chained operators..
                if grid[i][j] == 1 \
                    and (float(i+1) - SQUARE_SIDE/2.0) > self.x > (float(i) - SQUARE_SIDE/2.0)\
                    and (float(j+1) - SQUARE_SIDE/2.0) > self.y > (float(j) - SQUARE_SIDE/2.0):

                    self.num_collisions += 1
                    return False

        return True



    #TODO: change moving model, it is steering wheel for now which is wrong!!
    def move(self,  steering, distance, tolerance = 0.001, max_steering_angle = pi / 4.0):
        """
        Move the robot using bicycle model from Udacity class.
        @param steering front wheel steering angle
        @param distance distance to be driven
        """


        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0


        # make a new copy
        res = Robot()
        # TODO: not add new variables
        res.length            = self.length
        res.steering_noise    = self.steering_noise
        res.distance_noise    = self.distance_noise
        res.measurement_noise = self.measurement_noise
        res.sonar_noise = self.sonar_noise
        res.num_collisions    = self.num_collisions
        res.num_steps         = self.num_steps + 1

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise) if distance > 0 else 0.0

        print "Rotating by ",steering2, " with distance2=",distance2, " Wanted ",steering2, " ",distance2

        if distance > 0.0:
            # Execute motion
            turn = tan(steering2) * distance2 / res.length

            if abs(turn) < tolerance:

                # approximate by straight line motion

                res.x = self.x + (distance2 * cos(self.orientation))
                res.y = self.y + (distance2 * sin(self.orientation))
                res.orientation = (self.orientation + turn) % (2.0 * pi)

            else:
                # approximate bicycle model for motion
                radius = distance2 / turn
                cx = self.x - (sin(self.orientation) * radius)
                cy = self.y + (cos(self.orientation) * radius)
                res.orientation = (self.orientation + turn) % (2.0 * pi)
                res.x = cx + (sin(res.orientation) * radius)
                res.y = cy - (cos(res.orientation) * radius)
        else:
            # Rotational only move
            print "Rotating"
            res.x = self.x #TODO: add noise?
            res.y = self.y #TODO: add noise?
            res.orientation = (self.orientation + steering2) % (2.0 * pi)

        # check for collision
        # res.check_collision(grid)

        return res

    def sense_field(self, grid):
        disc_x, disc_y = int(self.x + SQUARE_SIDE/2.0), int(self.y + SQUARE_SIDE/2.0)
        return grid[disc_x][disc_y]

    def sense_gps(self):
        """ Returns estimation for position (GPS signal) """
        self.num_steps += SENSE_GPS_ACTIONS

        return [random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise)]

    def sense_sonar(self, grid):
        """
        Returns distance to wall using 128bit precision floats
        """
        tolerance_a = np.float128(1e-5)
        max_a = np.float128(1e4)
        found = False

        def is_hit(x, y):
            tolerance_xy = np.float128(1e-4) # will check nearby
            exact_hit = grid[int(x)][int(y)] == 1
            hit_right = int(x) < (len(grid)-1) and grid[int(x)+1][int(y)] == 1 and (x - int(x)) > (SQUARE_SIDE-tolerance_xy)
            hit_left = int(x) > 0 and grid[int(x)-1][int(y)] == 1 and (x - int(x)) < tolerance_xy
            hit_top = int(y) < (len(grid[0])-1) and grid[int(x)][int(y)+1] == 1 and (y - int(y)) > (SQUARE_SIDE-tolerance_xy)
            hit_bottom = int(y) > 0 and grid[int(x)][int(y)-1] == 1 and (y - int(y)) < tolerance_xy
            return exact_hit or hit_right or hit_left or hit_top or hit_bottom


        x_min_col, y_min_col = [np.float128(0), np.float128(0), np.float128(1e100)], [np.float128(0), np.float128(0),
                                                                                      np.float128(1e100)]

        x, y = np.float128(self.x + SQUARE_SIDE/2.0), np.float128((self.y + SQUARE_SIDE/2.0))
        logger.info(("robot:",x," ",y," ",self.orientation))
        x_disc, y_disc = int(x), int(y)

        orient_x = np.float128(np.cos(np.float128(self.orientation)))
        orient_y = np.float128(np.sin(np.float128(self.orientation)))
        a = np.float128(np.tan(np.float128(self.orientation)))
        b = np.float128(y - a*x)

        logger.info(("A=", a," B=", b))

        # Find collisions with x walls
        for i in xrange(0, len(grid)):

            if a > max_a:
                cross_x, cross_y = np.float128(float(i)+1e-10), np.float128(self.y)
            else:
                cross_x, cross_y = np.float128(float(i)+1e-10), np.float128(a*(float(i)+1e-10) + b)

            if cross_x < 0.0 or cross_x > len(grid)*SQUARE_SIDE or cross_y < 0.0 or cross_y > len(grid[0])*SQUARE_SIDE:
                continue

            diff_x, diff_y = np.float128(cross_x - x), np.float128(cross_y-y)
            if orient_x*diff_x + orient_y*diff_y > 0:
                if is_hit(cross_x, cross_y) and (diff_x**2 + diff_y**2) < x_min_col[2]:
                    x_min_col = [cross_x, cross_y, (diff_x**2 + diff_y**2)]
                    found = True

        # Find collisions with y walls
        for i in xrange(0, len(grid[0])):
            # Check if line is almost parallel to the axis
            if abs(a) > tolerance_a:
                cross_x, cross_y = np.float128((float(i)+np.float128(1e-10) - b)/a), \
                                   np.float128(float(i)+1e-10)
            else:
                cross_x, cross_y = np.float128(self.x), np.float128(float(i)+1e-10)


            if cross_x < 0.0 or cross_x > len(grid)*SQUARE_SIDE or cross_y < 0.0 or cross_y > len(grid[0])*SQUARE_SIDE:
                continue


            diff_x, diff_y = np.float128(cross_x - x), np.float128(cross_y - y)
            if orient_x*diff_x + orient_y*diff_y > 0:
                if is_hit(cross_x, cross_y) and (diff_x**2 + diff_y**2) < y_min_col[2]:
                    y_min_col = [cross_x, cross_y, (diff_x**2 + diff_y**2)]
                    found = True

        if not found:
            raise KrakrobotException("Something went wrong with sonar - not found wall! Note: boundary should be walled")

        return float(sqrt(min(x_min_col[2], y_min_col[2])))



    def __repr__(self):
        # return '[x=%.5f y=%.5f orient=%.5f]'  % (self.x, self.y, self.orientation)
        return '[%.5f, %.5f]'  % (self.x, self.y)
