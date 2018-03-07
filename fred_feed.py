import subprocess
from std_msgs.msg import String
from Franka import FrankaCustom
import matplotlib.pyplot as plt
import numpy as np

# Importing Chess team - Ben Greenberg's classes
from franka.franka_control import FrankaControl
from franka.franka_control_ros import FrankaRos


# initiating FrankaControl
arm = FrankaControl(debug_flag=True) 
arm_ros = FrankaRos()
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
	# camera_point = fred.get_mouth_pos() 
	# m = [0.0014412346027600322, -0.0064285714285713625, 6.829108391583492e-07]
	# c = [-0.17613191331662731, 0.590942857142845, 0.34574058675699332]
	# scale = [m, c]
	# go = raw_input("Would you like to go to that camera point? [Y/n]: ")
	# print(go)
	# if (go == '' or go.lower() == 'y'):
	# 	end = fred.convert_pt(camera_point, scale)
	# 	end = [str(i) for i in end]
	# 	start = fred.get_end_effector_pos()
	# 	print('start:', start)
	# 	print('end:', end)
	# 	certain = raw_input("You certain? [Y/n]: ")
	# 	if (certain == '' or certain.lower() == 'y'):
	# 		arm.move_absolute(end)
	# fred.get_end_effector_pos()

	arm_ros.example_movement()

	# arm.move_absolute(['0.162', '0.192', '0.658'])
	# arm.move_relative('0.0', '0.1', '0.0')
	# fred.get_end_effector_pos()
	# scale = fred.manual_calibrate()
	# print("Yay!", scale)

fred_feed()