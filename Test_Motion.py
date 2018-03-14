# !/usr/bin/env python

from math import sqrt
from baxter_control import BaxterControl
import baxter_interface
import roslib
import sys
import rospy
from message_filters import Subscriber
from geometry_msgs.msg import Point
import argparse

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest


class ReactiveControl:
    def __init__(self, baxter_control):
        self.baxter_control = baxter_control
        self.mouth_subscription = None
        self.food_subscription = None
        self.food_point = None
        self.mouth_point = None
        self.distance = None
        self.direction = None
        self.speed = None
        self.kinematic_control = KinematicControl()

    def turn_on(self):
        if self.mouth_subscription is None and self.food_subscription is None:
            self.mouth_subscription = rospy.Subscriber("/mouth_xyz_astra", Point, self.__mouth_callback__)
            self.food_subscription = rospy.Subscriber("/mouth_xyz_astra", Point, self.__food_callback__)

    def turn_off(self):
        if self.mouth_subscription is not None and self.food_subscription is not None:
            self.mouth_subscription.unregister()
            self.food_subscription.unregister()
            self.food_subscription = None
            self.mouth_subscription = None

    # TODO: Testing for right speed ratios
    def set_speed(self):
        if self.__points_valid__():
            self.speed = MIN_SPEED + self.distance / SPEED_CONST

    def __points_valid__(self):
        return self.food_point is not None and self.mouth_point is not None

    # Callback for mouthxyz changes. Make sure we subscribe with this as callback!
    def __mouth_callback__(self, mouth_point):
        self.mouth_point = mouth_point
        self.__update_values__()

    # Callback for foodxyz changes. Make sure we subscribe with this as callback!
    def __food_callback__(self, food_point):
        self.food_point = food_point
        self.__update_values__()

    def __set_distance__(self):
        if self.__points_valid__():
            self.distance = get_distance_between_points(self.food_point, self.mouth_point)

    def __set_direction__(self):
        if self.__points_valid__():
            self.direction = get_direction_between_points(self.food_point, self.mouth_point)

    def __update_values__(self):
        # self.set_distance()
        # self.set_direction()
        # self.set_speed()
        self.__update_robot_motion__()

    def __update_robot_motion__(self):
        self.baxter_control.set_ee_pos(self.mouth_point.x, self.mouth_point.y, self.mouth_point.z)




def main(args):

    baxter_control = BaxterControl()
    rc = ReactiveControl(baxter_control)

    rospy.init_node('Reactive', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    print("Test Motion")
    main(sys.argv)