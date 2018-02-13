import numpy as np
import message_filters
import subprocess
import rospy
from std_msgs.msg import String
from franka.franka_control import FrankaControl

# initiating caller (https://github.com/nebbles/DE3-ROB1-CHESS/blob/master/franka/caller.py#L9)
arm = FrankaControl(debug_flag=True) # we set the flag true to get prints to the console about what Caller is doing

# franka status

## MOVING Functions ---------------------------
# danger function protocol
def danger():
	try:
		# arm moves back by 10cm
		arm.move_relative(-0.1, 0.0, 0.0)
	except:
		print ("Cannot execute danger protocol")

# default protocol function
def default():
	# moving to safe position
	arm.move_absolute([0.0, 0.0, 0.0]) #< still not set

# temp human in loop function
def human_in_loop():
	arm.move_relative(0.1, 0.0, 0.0)


## SUBSCRIBING MOUTH NODE --------------------------
# callback function used by the subscriber
def mouth_node_sub(callback_func):
	# initialise the node
	rospy.init_node("chat_subscriber_node", anonymous=True)
	# subscribe to the chat topic and attach the callback function
	rospy.Subscriber("/mouthxy", String, callback_func)
	# loop forever
	rospy.spin()

def show_message(message):
    print message.data

## MAIN --------------------------
def main():
	try:
		mouth_node_sub(show_message)
	except:
		pass

	for i in range(3):
		human_in_loop()

main()