#Note : work in progress

from defines import *
from robot_controller import RobotController
import random
from math import pi
from math import atan2
from collections import Counter


class Turner(RobotController):
    """ Exemplary robot controller choosing to go where it wasn't before :) """

    STATE_FINDING_POSITION = 0 # Robot is localizing itself on GPS to update map_visited
    STATE_DECIDE_NEXT = 1 # Robot decides where to goes
    STATE_FOUND_GOAL = 2 # Robot found goal, yell finish!
    STATE_SCANNING = 3 # Robot doesnt know about fields nearby if there are walls or open, scanning

    def get_discrete_position(self):
        return int(self.x+0.5) , int(self.y+0.5)

    def init(self, starting_position, steering_noise, distance_noise, sonar_noise,
             measurement_noise, speed, turning_speed, gps_delay, execution_cpu_time_limit):
        self.speed = speed
        self.turn_speed = turning_speed
        self.command_queue = []
        self.last_distance = 0.0

        self.state = Turner.STATE_DECIDE_NEXT
        self.state_helper = 0


        self.x = starting_position[0]
        self.y = starting_position[1]
        self.angle = 0 # 0 degrees to x axis

        self.map_visited = {}
        self.map_visited[self.get_discrete_position()] = 1 # <=> Open and visited once

    def act(self):
        # We need to plan actions if we are out of actions in out command_queue
        while len(self.command_queue) ==0 or self.command_queue[0][0] == "STATE_CHANGE":


            # Add state change resolving
            if len(self.command_queue) != 0 and self.command_queue[0][0] == "STATE_CHANGE":
                self.state = self.command_queue[0][1]
                self.command_queue.pop(0)
                self.state_helper = 0

            x_disc, y_disc = self.get_discrete_position()

            # We are standing on a field, in arbitrary angle and deciding what next
            if self.state == Turner.STATE_DECIDE_NEXT:
                print "STATE_DECIDE_NEXT"

                neigh = [ (x_disc+1, y_disc), (x_disc-1, y_disc), (x_disc, y_disc+1), (x_disc, y_disc-1)]

                # Check if we have neighbouring unknown field
                if not all((x in self.map_visited for x in neigh)):

                    self.command_queue.append([SENSE_SONAR])
                    self.command_queue.append(["STATE_CHANGE", Turner.STATE_SCANNING])

                else:
                    neigh = [ (x_disc+1, y_disc), (x_disc-1, y_disc), (x_disc, y_disc+1), (x_disc, y_disc-1)]
                    find_min = [(self.map_visited[x], x) for x in neigh]
                    find_min.sort()
                    print "Result of scanning, or known before: ", find_min

                    goal = find_min[0][1] # Get minimum goal
                    print "Goal is ", goal
                    vector = (goal[0] - x_disc, goal[1] - y_disc)
                    print "Vector is ", vector
                    # Angle calculation + change of coordinates
                    angle = atan2(-vector[0], vector[1]) / pi * 180.0 + 90.0




                    print "Current angle is ",self.angle, "rotation by ",(angle - self.angle), " to ",angle
                    rotation_ticks = int((angle - self.angle)/180.0 * pi / TICK_ROTATE)
                    if rotation_ticks != 0 : self.command_queue.append([TURN, rotation_ticks])
                    self.angle = angle
                    self.command_queue.append([MOVE, int(1.0/TICK_MOVE)])
                    self.command_queue.append(["STATE_CHANGE", Turner.STATE_FINDING_POSITION])

                self.state_helper = 0
            # We are scanning fields to see what is there, we have just observed something
            elif self.state == Turner.STATE_SCANNING:
                print "Scanning on ",x_disc, " ",y_disc

                import math
                # Directional cosinus
                # Change of coordinates for convenience
                vector = (math.cos((self.angle - 90.0)/180.0 * pi), math.sin((self.angle - 90.0)/180.0 * pi))
                print "Vector = ", vector
                scanned = (x_disc-round(vector[1]), y_disc + round(vector[0]))
                print "Scanned ", scanned
                print "Distance scanned ",self.last_distance
                if (x_disc-round(vector[1]+0.01), y_disc + round(vector[0]+0.01)) not in self.map_visited:
                    if self.last_distance < 0.9:
                        # Strange indexing because x runs vertically and y runs horizontally
                        # Set big number so that it won't be visited
                        self.map_visited[(x_disc-round(vector[1]+0.01), y_disc + round(vector[0]+0.01))] = 1000
                    else:
                        self.map_visited[(x_disc-round(vector[1]+0.01), y_disc + round(vector[0]+0.01))] = 0

                self.state_helper += 1


                # It means that we have reached the last rotation
                if self.state_helper == 4:
                    self.command_queue.append(["STATE_CHANGE", Turner.STATE_DECIDE_NEXT])
                else:
                    self.command_queue.append([TURN, int(0.5*pi / TICK_ROTATE)])
                    self.angle = (self.angle + 90.0) % 360

                    print "Angle after rotation ", self.angle
                    self.command_queue.append([SENSE_SONAR])

            # Find next move
            elif self.state == Turner.STATE_FINDING_POSITION:
                # Scanning for distance
                if self.state_helper == 0:
                    # Keep (accumulated so far, (position_x_accumulated, position_y_accumualted))
                    self.state_helper = [0, [0., 0.]]
                    self.command_queue.append([SENSE_GPS])
                elif self.state_helper[0] < 3:
                    self.state_helper[0] += 1
                    self.state_helper[1][0] += self.x
                    self.state_helper[1][1] += self.y
                    self.command_queue.append([SENSE_GPS])
                else:
                    self.x = self.state_helper[1][0]/3.0
                    self.y = self.state_helper[1][1]/3.0

                    print "New position ",self.x, self.y

                    x_disc, y_disc = self.get_discrete_position()

                    self.map_visited[(x_disc, y_disc)] += 1
                    self.command_queue.append([SENSE_FIELD])
                    self.command_queue.append(["STATE_CHANGE", Turner.STATE_DECIDE_NEXT])
            # Finish
            elif self.state == Turner.STATE_FOUND_GOAL:
                self.command_queue = [[FINISH]]


        # Return next command
        c = self.command_queue.pop(0)
        return c


    def on_sense_gps(self, x, y):
        print "Sensed ",x, y
        self.x = x
        self.y = y

    def on_sense_sonar(self, distance):
        self.last_distance = distance

    def on_sense_field(self, field_type, field_parameter):

        if field_type == MAP_GOAL:
            self.command_queue = [[FINISH]]
