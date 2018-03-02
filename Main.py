import rospy
from std_msgs.msg import String
import message_filters
from message_filters import Subscriber

# this is the main code that executes operations and acquires data from other files
# before running main make sure you have all the other files running

class Main:
    def __init__(self):
        self.foodxyz = Subscriber("PATH TO FOOD TOPIC ", String)
        self.mouthxyz = Subscriber("PATH TO MOUTH TOPIC ", String)

    def gotofood(self):

    def pickupfood(self):

    def gotomouth(self):

    def release(self):

