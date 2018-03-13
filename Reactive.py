# !/usr/bin/env python

from math import sqrt
import baxter_interface
import roslib
import sys
import rospy
from message_filters import Subscriber
from geometry_msgs.msg import Point

SPEED_CONST = 10
MIN_SPEED = 10


def get_difference_between_points(a, b):
    return b.x - a.x, b.y - a.y, b.z - a.z


def get_distance_between_points(a, b):
    dx, dy, dz = get_difference_between_points(a, b)
    return sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))


def get_direction_between_points(a, b):
    dx, dy, dz = get_difference_between_points(a, b)
    mag = get_distance_between_points(a, b)
    return [dx / mag, dy / mag, dz / mag]


# Move endpoint to where we want it by solving IK and using joint velocities. Hopefully this works XD
class KinematicControl:
    def __init__(self):
        self.ik_solution = None
        self.required_joint_velocities = None

    def set_desired_endpoint(self, desired_endpoint):
        # TODO: Use baxter IK Service to set self.ik_solution to required joint movements for desired_endpoint
        self.ik_solution = None # = solution. should be dict {joint_name : required_angle_change}
        pass

    def set_joint_velocities(self, speed):
        self.required_joint_velocities = {}
        max_joint_change = max([abs(angle) for angle in self.ik_solution.values()])
        for joint, required_angle_change in self.ik_solution:
            max_joint_change_ratio = required_angle_change / max_joint_change
            self.required_joint_velocities[joint] = max_joint_change_ratio

    # speed should be between 0.0 and 0.5 (i.e. is ratio of max speed)
    def move_to_desired_endpoint(self, desired_endpoint, speed):
        limb = baxter_interface.Limb('right')
        limb.set_joint_velocities(self.required_joint_velocities)


class ReactiveControl:
    def __init__(self):
        self.food_point = None
        self.mouth_point = None
        self.distance = None
        #self.direction = None
        self.speed = None
        self.kinematic_control = KinematicControl()
        self.mouth_xyz_sub = Subscriber("/mouth_status", Point, queue_size = 10)

    def points_valid(self):
        return self.food_point is not None and self.mouth_point is not None

    # Callback for mouthxyz changes. Make sure we subscribe with this as callback!
    def mouth_callback(self,  mouth_point):
        self.mouth_point = mouth_point
        self.update_values()

    # Callback for foodxyz changes. Make sure we subscribe with this as callback!
    def food_callback(self, food_point):
        self.food_point = food_point
        self.update_values()

    def set_distance(self):
        if self.points_valid():
            self.distance = get_distance_between_points(self.food_point, self.mouth_point)

    def set_direction(self):
        if self.points_valid():
            self.direction = get_direction_between_points(self.food_point, self.mouth_point)

    # TODO: Testing for right speed ratios
    def set_speed(self):
        if self.points_valid():
            self.speed = MIN_SPEED + self.distance / SPEED_CONST

    def update_values(self):
        self.set_distance()
        self.set_direction()
        self.set_speed()
        self.update_robot_motion()

    def update_robot_motion(self):
        self.kinematic_control.move_to_desired_endpoint(self.mouth_point, self.speed)


