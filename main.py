import subprocess
import rospy
from std_msgs.msg import String
from franka.franka_control import FrankaControl
import matplotlib.pyplot as plt
import numpy as np
from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score

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
def mouth_node_sub(callback_func, loop_tf = True):
	try:
		loop =True
		while (loop):
			# initialise the node
			rospy.init_node("read_mouth_location", anonymous=True)
			# subscribe to the chat topic and attach the callback function
			rospy.Subscriber("/mouthxyz", String, callback_func)
			# loop forever
			loop = loop_tf
	except: 
		print("mouth_node_sub error")
		pass

def return_uvw(mouthuvw):
	print(mouthuvw.data)
	return mouthuvw.data

## CALIBRATION ---------------------------
def record_data_pt(uvw_pt, xyz_pt):
	uvw_pt.append(mouth_node_sub(return_uvw))
	xyz_pt.append(arm.get_end_effector_pos())


## MAIN --------------------------
def main():
	mouth_node_sub(return_uvw)

	#arm.get_end_effector_pos()

main()