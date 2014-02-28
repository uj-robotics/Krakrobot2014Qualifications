
from defines import *
from robot_controller import RobotController
import random
from math import pi

class Rotator(RobotController):
    """ Exemplary robot controller rotating by 90 degrees :) """
    STATE_ROTATE = 0

    def init(self, starting_position, steering_noise, distance_noise, sonar_noise, measurement_noise, speed, turning_speed,gps_delay,execution_cpu_time_limit):


        self.state = Rotator.STATE_ROTATE
        self.speed = speed
        self.turn_speed = turning_speed
        self.command_queue = []
        self.last_distance = 0.0
        self.x = 0
        self.y = 0
        self.state_helper = 1

    def act(self):
        if len(self.command_queue) == 0:
            if self.state == Rotator.STATE_ROTATE:
                self.command_queue.append([TURN, self.state_helper * int((0.5*pi)/ TICK_ROTATE )]) # Rotate by 90 deegres
                self.state_helper *= -1
        return self.command_queue.pop(0)

    def on_sense_gps(self, x, y):
        self.x = x
        self.y = y

    def on_sense_sonar(self, distance):
        self.last_distance = distance


    def on_sense_field(self, field_type, field_parameter):
        if field_type == MAP_GOAL:
            self.phase = MAP_GOAL

