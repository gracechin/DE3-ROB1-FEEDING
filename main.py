import rospy
from std_msgs.msg import String
import message_filters
from message_filters import Subscriber
from baxter_control import BaxterControl
from reactive import ReactiveControl

# this is the main code that executes operations and acquires data from other files
# before running main make sure you have all the other files running

class Main:
    def __init__(self):
    	self.reactive = ReactiveControl()
    	self.baxter_control = BaxterControl()
    	# TODO:
        #self.foodxyz = Subscriber("PATH TO FOOD TOPIC ", String)
        #self.mouthxyz = Subscriber("PATH TO MOUTH TOPIC ", String)

    def gotofood(self):

    def pickupfood(self):
    	self.baxter_control.playback_scooping()

    def gotomouth(self):
        self.reactive.turn_on()

    def release(self):

def main(args):
	


if __name__ == '__main__':
    print("Reactive Control")
    main(sys.argv)

