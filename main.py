import subprocess
import rospy
from std_msgs.msg import String
from franka.franka_control import FrankaControl

# initiating FrankaControl
arm = FrankaControl(debug_flag=True) 

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


## SUBSCRIBING MOUTH NODE --------------------------
# callback function used by the subscriber
def mouth_node_sub(callback_func):
	try:
		# initialise the node
		rospy.init_node("chat_subscriber_node", anonymous=True)
		# subscribe to the chat topic and attach the callback function
		rospy.Subscriber("/mouthxy", String, callback_func)
		# loop forever
		rospy.spin()
	except: 
		print("mouth_node_sub error")
		pass

def show_message(message):
    print(message.data)

## MAIN --------------------------
def main():
	mouth_node_sub(show_message)

	#arm.get_status()

main()