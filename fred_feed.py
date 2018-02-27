import subprocess
from std_msgs.msg import String
from franka_custom import FrankaCustom
from franka.franka_control import FrankaControl
from astra import MouthPos
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
	test = False
	scale = fred.calibrate()
	test = True

	while test:
		camera_point = fred.get_mouth_pos()
		print("camera_point:", camera_point)
		go = raw_input("Would you like to go to that camera point? [Y/n]: ")
		print(go)
            if (go == '' or go.lower() == 'y'):
                end = fred.convert_pt(camera_point)
                start = fred.get_end_effector_pos
                print('start:', start)
                print('end:', end)
                certain = raw_input("You certain? [Y/n]: ")
                if (certain == '' or certain.lower() == 'y'):
                	arm.move_absolute[end]
                else: pass
            elif (record.lower() == 'n'): pass
            elif (record.lower() == 'q'): sys.quit()
            else: print("Invalid response.")

    else: sys.quit()


fred_feed()