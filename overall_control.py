# !/usr/bin/env python

import rospy
from std_msgs.msg import String
import message_filters
from message_filters import Subscriber
from baxter_control import BaxterControl
from reactive import ReactiveControl
from perception_sub import PerceptionSub
from reactive import ReactiveControl


# Main class for robot control
class OverallControl:
    def __init__(self):
        self.candy_subscription = None
        self.baxter_control = BaxterControl()
        self.reactive_control = ReactiveControl(baxter_control)
        # self.recorded_control = None # TODO: Recorded class here

    def begin_execution(self):
		self.__begin_candy_subscription__()

    def stop_execution(self):
		self.__unregister_candy_subscription__()

    def __begin_candy_subscription__(self):
        if self.candy_subscription is None:
            self.candy_subscription = rospy.Subscriber("/candy", String, self.__candy_callback__)

    def __unregister_candy_subscription__(self):
        if self.candy_subscription is not None:
            self.candy_subscription.unregister()
            self.candy_subscription = None

    def __candy_callback__(self, candy_state):
        if candy_state == 'True':
            self.__switch_to_reactive__()
        else:
            self.__unregister_candy_subscription__()
            # TODO: Start recording. During this time we should also stop candy subscription.
            self.__switch_to_recording__()
            self.__begin_candy_subscription__()

    def __switch_to_reactive__(self):
        self.reactive_control.turn_on()
        # TODO: Make sure recorded is off

    def __switch_to_recording__(self):
        self.reactive_control.turn_off()
        self.baxter_control.playback_scooping()



def main(args):
	oc = OverallControl()
    ps = PerceptionSub()
    rc = ReactiveControl(baxter_control)

	oc.begin_execution()

    rospy.init_node('overall', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    print("Overall Control")
    main(sys.argv)
