from defines import *
from robot_controller import RobotController
import random

class ForwardTurningRobotController(RobotController):
    """ Exemplary robot controller """
    def init(self, starting_position, steering_noise, distance_noise, measurement_noise, speed, turning_speed, execution_cpu_time_limit):
        self.speed = speed

    def act(self):
        return MOVE, 0.0, self.speed


    def on_sense_sonar(self, dist):
        pass

    def on_sense_field(self, file_type, file_parameter):
        pass

    def on_sense_gps(self, x, y):
        pass