import subprocess
from std_msgs.msg import String
from franka_custom import FrankaCustom
from franka.franka_control import FrankaControl
from astra import MouthPos
import matplotlib.pyplot as plt
import numpy as np


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


def return_uvw(mouthuvw):
	print(mouthuvw.data)
	return mouthuvw.data

## CALIBRATION ---------------------------
def record_data_pt(uvw_pt, xyz_pt):
	uvw_pt.append(mouth_node_sub(return_uvw))
	xyz_pt.append(arm.get_end_effector_pos())


## MAIN --------------------------
def fred_feed():
	fred = FrankaCustom()
	mouthxyz = fred.calibrate()


	#arm.get_end_effector_pos()


fred_feed()