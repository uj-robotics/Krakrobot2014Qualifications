
from defines import *
from robot_controller import RobotController
import random

class OmitCollisions(RobotController):
    """ Exemplary robot controller omitting collisions """
    STATE_FORWARD = 0
    STATE_LOOK_FOR_SPACE = 1

    def init(self, starting_position, steering_noise, distance_noise, sonar_noise, measurement_noise, speed, turning_speed,gps_delay,execution_cpu_time_limit):


        self.phase = OmitCollisions.STATE_LOOK_FOR_SPACE
        self.speed = speed
        self.turn_speed = turning_speed
        self.command_queue = []
        self.last_distance = 0.0

    def act(self):
        if len(self.command_queue) == 0:
            if self.phase == OmitCollisions.STATE_LOOK_FOR_SPACE:
                self.command_queue.append([TURN, random.randint(-1, 1)* 10])
                self.command_queue.append([SENSE_SONAR])
                self.command_queue.append([WRITE_CONSOLE, "Hello"])
            elif self.phase == MAP_GOAL:
                self.command_queue.append([FINISH])
            else:
                self.command_queue.append([MOVE, 1])
                self.command_queue.append([SENSE_SONAR])
                self.command_queue.append([SENSE_FIELD])

        return self.command_queue.pop(0)

    def on_sense_sonar(self, distance):
        self.last_distance = distance
        if distance < 0.01:
            self.phase = OmitCollisions.STATE_LOOK_FOR_SPACE
        else:
            self.phase = OmitCollisions.STATE_FORWARD


    def on_sense_field(self, field_type, field_parameter):
        if field_type == MAP_GOAL:
            self.phase = MAP_GOAL

