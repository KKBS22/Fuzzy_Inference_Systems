import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt

from skfuzzy import control as ctrl


# Atecedent objects hold universe variables and membership
distance_obj = ctrl.Antecedent(np.arange(0, 10, 0.5), 'distance')
angle_obj = ctrl.Antecedent(np.arange(0, 90, 1), 'angle')


distance_obj['near'] = fuzz.trapmf(distance_obj.universe, [0, 0.5, 1, 1.5])
distance_obj['far'] = fuzz.trapmf(distance_obj.universe, [1, 1.5, 6, 6.5])
distance_obj['veryfar'] = fuzz.trapmf(distance_obj.universe, [6, 6.5, 10, 10])


angle_obj['small'] = fuzz.trapmf(angle_obj.universe, [0, 5, 10, 15])
angle_obj['medium'] = fuzz.trapmf(angle_obj.universe, [10, 15, 45, 50])
angle_obj['large'] = fuzz.trapmf(angle_obj.universe, [45, 50, 90, 90])


distance_obj.view()
angle_obj.view()


# Consequent objects hold universe variables and membership

speed_obj = ctrl.Consequent(np.arange(0, 5, 0.2), 'speed', 'centroid')
steering_obj = ctrl.Consequent(np.arange(0, 90, 1), 'steering', 'centroid')

speed_obj['slowspeed'] = fuzz.trapmf(speed_obj.universe, [0, 0.2, 0.4, 0.6])
speed_obj['mediumspeed'] = fuzz.trapmf(speed_obj.universe, [0.4, 0.6, 2, 2.2])
speed_obj['fastspeed'] = fuzz.trapmf(speed_obj.universe, [2, 2.2, 4, 4.2])
speed_obj['maxspeed'] = fuzz.trapmf(speed_obj.universe, [4, 4.2, 5, 5])


sigma = 10
steering_obj['mildturn'] = fuzz.gaussmf(steering_obj.universe, 0, sigma)
steering_obj['sharpturn'] = fuzz.gaussmf(steering_obj.universe, 45, sigma)
steering_obj['verysharpturn'] = fuzz.gaussmf(
    steering_obj.universe, 90, sigma)

speed_obj.view()
steering_obj.view()


# defining the rules
rule_one = ctrl.Rule(distance_obj['near'] & angle_obj['small'],
                     (speed_obj['slowspeed'], steering_obj['verysharpturn']))
rule_two = ctrl.Rule(distance_obj['near'] & angle_obj['medium'],
                     (speed_obj['mediumspeed'], steering_obj['sharpturn']))
rule_three = ctrl.Rule(distance_obj['near'] & angle_obj['large'],
                       (speed_obj['mediumspeed'], steering_obj['mildturn']))
rule_four = ctrl.Rule(distance_obj['far'] & angle_obj['small'],
                      (speed_obj['mediumspeed'], steering_obj['sharpturn']))
rule_five = ctrl.Rule(distance_obj['far'] & angle_obj['medium'],
                      (speed_obj['mediumspeed'], steering_obj['mildturn']))
rule_six = ctrl.Rule(distance_obj['far'] & angle_obj['large'],
                     (speed_obj['maxspeed'], steering_obj['mildturn']))
rule_seven = ctrl.Rule(distance_obj['veryfar'] & angle_obj['small'],
                       (speed_obj['maxspeed'], steering_obj['mildturn']))
rule_eight = ctrl.Rule(distance_obj['veryfar'] & angle_obj['medium'],
                       (speed_obj['maxspeed'], steering_obj['mildturn']))
rule_nine = ctrl.Rule(distance_obj['veryfar'] & angle_obj['large'],
                      (speed_obj['maxspeed'], steering_obj['mildturn']))


obstacle_ctrl = ctrl.ControlSystem(
    [rule_one, rule_two, rule_three, rule_four, rule_five, rule_six, rule_seven, rule_eight, rule_nine])
obstacle_avoidance = ctrl.ControlSystemSimulation(obstacle_ctrl)
# ToDo Mamdani..
# Example 1
obstacle_avoidance.input['distance'] = 1
obstacle_avoidance.input['angle'] = 90

# Crunch the numbers
obstacle_avoidance.compute()

print(obstacle_avoidance.output['speed'])
speed_obj.view(sim=obstacle_avoidance)


print(obstacle_avoidance.output['steering'])
steering_obj.view(sim=obstacle_avoidance)


# Example 2
obstacle_avoidance.input['distance'] = 7
obstacle_avoidance.input['angle'] = 1

obstacle_avoidance.compute()

print(obstacle_avoidance.output['speed'])
speed_obj.view(sim=obstacle_avoidance)


print(obstacle_avoidance.output['steering'])
steering_obj.view(sim=obstacle_avoidance)

print("Ting membership")


# Todo Simulation
class Obstacle:
    def __init__(self, xPos, yPos):
        self.x_pos = xPos
        self.y_pos = yPos
        pass


class Vehicle:
    position_hist_dict = {}

    def __init__(self, startPos, endPos):
        self.start_pos = startPos
        self.end_pos = endPos
        pass

    def calculate_distance(self):
        pass


class Layout:

    def __init__(self, width, length, obstacleCount, startPos, endPos):
        self.width = width
        self.length = length
        self.obstacle_nos = obstacleCount

    def generate_path(self):
        pass
