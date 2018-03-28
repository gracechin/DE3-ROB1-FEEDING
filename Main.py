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
    # while loop
    # substibe to food: is food present
    # subscribe to food: xyz
    # call franka move file to xyz
    # call pickupfood()

    def pickupfood(self):

    # call gripper file
    # call gripper force detect if force = present then call gotomouth()

    def gotomouth(self):
    # while loop
    # if face present call franka move file to xyz but 10cm away
    # if mouth open call franka move file to xyz of mouth
    # call gripper force detect if detected call gripper release



