# !/usr/bin/env python

import rospy
from std_msgs.msg import String
import message_filters
from message_filters import Subscriber
from baxter_control import BaxterControl
from reactive import ReactiveControl
from perception_sub import PerceptionSub
from Reactive import ReactiveControl


class OverallControl:
	def __init__(self):
		self.candy_subscription = None

	def turn_on_candy(self):
		self.candy_subscription = rospy.Subscriber("/candy", String, self.__candy_callback__)

	def __candy_callback__(self, candy_state):
		if candy_state == 'True':
			rc.turn_on()
		else: 
			print(C)
	def mouth(self, M):()
		print(M)

    def gotomouth(self):
        self.reactive.turn_on()

    def release(self):


def main(args):
	ps = PerceptionSub()
	rc = ReactiveControl()
	rospy.init_node('overall', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
    print("Overall Control")
    main(sys.argv)

