import subprocess
from std_msgs.msg import String
from Franka import FrankaCustom
from franka.franka_control import FrankaControl
import matplotlib.pyplot as plt
import numpy as np


# initiating FrankaControl
arm = FrankaControl(debug_flag=True) 
fred = FrankaCustom()

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
	scale = [[0.0015370193568726351, -0.058249999999999871, 9.3565241303452144e-05], [-0.45684233493064297, 10.913749999999977, 0.32972927265227553]]
	camera_point = fred.get_mouth_pos()
	go = raw_input("Would you like to go to that camera point? [Y/n]: ")
	print(go)
	if (go == '' or go.lower() == 'y'):
		end = fred.convert_pt(camera_point, scale)
		end = [str(i) for i in end]
		start = fred.get_end_effector_pos()
		print('start:', start)
		print('end:', end)
		certain = raw_input("You certain? [Y/n]: ")
		if (certain == '' or certain.lower() == 'y'):
			arm.move_absolute(end)
	fred.get_end_effector_pos()
	# arm.move_absolute(['0.162', '0.192', '0.658'])
	# arm.move_relative('0.0', '0.1', '0.0')
	# fred.get_end_effector_pos()
	# scale = fred.calibrate()
	print("Yay!", scale)

fred_feed()