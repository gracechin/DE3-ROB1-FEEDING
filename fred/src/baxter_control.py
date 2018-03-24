#!/usr/bin/env python

import rospy
import rospkg
import baxter_interface
from baxter_interface import CHECK_VERSION
from joint_playback import Trajectory
import baxter_external_devices
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
    Float32MultiArray,
)
from message_filters import Subscriber
from trac_ik_python.trac_ik import IK # for inverse_kinematics
import ast
from os import path


## ENABLE ROBOT
# rosrun baxter_tools enable_robot.py -e

## SEPARATE TERMINAL RUN
# rosrun baxter_interface joint_trajectory_action_server.py --mode velocity
# python baxter_pub.py 

class BaxterControl:
	def __init__(self, arm="right"):
		self.default_arm = arm
		self.ee_face_forward = [-0.44540951393, 0.551357225282, -0.475534129977, 0.521040177201]

	def get_joints(self, limb=None): 
		if (limb==None): limb = self.default_arm
		publisher_name = 'baxter_' + limb + '_joints'
		ee_ori_sub = rospy.Subscriber(publisher_name, Float32MultiArray, self.return_msg)
		rospy.wait_for_message(publisher_name, Float32MultiArray)
		return msg

	def get_end_effector_ori(self, limb=None): 
		if (limb==None): limb = self.default_arm
		publisher_name = 'baxter_' + limb + '_ori'
		ee_ori_sub = rospy.Subscriber(publisher_name, Quaternion, self.return_msg)
		rospy.wait_for_message(publisher_name, Quaternion)
		return msg

	def get_end_effector_pos(self, limb=None): 
		if (limb==None): limb = self.default_arm
		publisher_name = 'baxter_' + limb + '_pos'
		ee_pos_sub = rospy.Subscriber(publisher_name, Point, self.return_msg)
		rospy.wait_for_message(publisher_name, Point)
		return msg

	def return_msg(self, message):
		global msg
		msg = message
		return msg

	def set_joint_angles(self, joints, limb=None):
		print('Moving there')
		'''Sets joint angles of Baxter 
		joints = list of the desired joint angles
		Baxter moves joints to the angles'''
		if (limb==None): limb = self.default_arm
		arm = baxter_interface.Limb(limb)
	 	jn = arm.joint_names()
		# print("before joint angles:", arm.joint_angles())
		positions = dict(zip(jn, joints))
		# print("desired joint angles:", positions)
		arm.move_to_joint_positions(positions, timeout=15.0, threshold=0.008726646)
		# print("after joint angles:", arm.joint_angles())
		rospy.sleep(4.)

	def calibrate_gripper(self, limb=None):
		if (limb==None): limb = self.default_arm
		gripper = baxter_interface.Gripper(limb, CHECK_VERSION)
		gripper.calibrate()

	def close_gripper(self, limb=None):
		if (limb==None): limb = self.default_arm
		gripper = baxter_interface.Gripper(limb, CHECK_VERSION)
		gripper.close()

	def open_gripper(self, limb=None):
		if (limb==None): limb = self.default_arm
		gripper = baxter_interface.Gripper(limb, CHECK_VERSION)
		gripper.calibrate()
		gripper.open()

	def set_end_effector_pos(self, x, y, z, qx=None, qy=None, qz=None, qw=None, limb=None):
		'''Sets end effector position of Baxter
		x, y, z = positions of the end effector
		qx, qy, qz, qw = orientation of the end effector (Quaternion)
		Change in Baxter end effector'''
		print('BaxterControl: set ee pos')
		if (qx == None): qx, qy, qz, qw = self.ee_face_forward[0], self.ee_face_forward[1], self.ee_face_forward[2], self.ee_face_forward[3]    
		if (limb==None): limb = self.default_arm
		urdf_str = rospy.get_param('/robot_description')
		# Defining the two frames of reference wanted on the robot
		ik_solver = IK("base", limb+"_gripper", urdf_string=urdf_str) 
		lb, up = ik_solver.get_joint_limits() 
		# Setting lower and upper limits to the joints
		ik_solver.set_joint_limits([-1.70168, -2.147, -3.05418, -0.05, -3.059, -1.5708, -3.059], 
			[1.70168, 1.047, 3.05418, 2.618, 3.059, 2.094, 3.059])

		# Minimises joint movement from current position
		# arm = baxter_interface.Limb(limb)
		# current_position = arm.joint_angles()
		# temp = []
		# dictlist = []
		# for key, value in current_position.iteritems():
		# 	temp = [key,value]
		# 	dictlist.append(temp[1])
		# # print current_position
		
		# ordered_joints = [dictlist[5], dictlist[6], dictlist[3], dictlist[4], dictlist[0], dictlist[1], dictlist[2]]
		# # print ordered_joints

		# ### SEEDING CURRENT JOINT POSITIONS INTO THE SOLVER
		# seed_state = ordered_joints
		seed_state = [0.0] * ik_solver.number_of_joints
		# Inserting desired points for the solver to solve
		# arm_ori = self.get_end_effector_ori()
		# arm_pos = self.get_end_effector_pos()
		# qx, qy, qz, qw = arm_ori.x, arm_ori.y, arm_ori.z, arm_ori.w
		bx = by = bz = 0.02
		brx = bry = brz = 9999.9
		# print(z)
		# z = arm_pos.z
		sol = str(ik_solver.get_ik(seed_state, x, y, z, qx, qy, qz, qw, bx, by, bz, brx, bry, brz)) # put these in as integers 
		print('sol', sol)
		# converting sol from a string to a list
		sol.replace('(','[');
		sol.replace(')',']');
		sol = ast.literal_eval(sol)
		# print('Desired position:', [x, y, z])
		# print('Desired joint Angles:', sol)
		# print('setting joint angles....')
		self.set_joint_angles(sol)
		return sol

	def playback_scooping(self):
	    # print("Getting robot state... ")
	    # rs = baxter_interface.RobotEnable(CHECK_VERSION)
	    # print("Enabling robot... ")
	    # rs.enable()
	    print("Running Replay. Ctrl-c to quit")
	    traj = Trajectory()
	    traj.parse_file(path.expanduser('/home/robin/catkin_ws/src/fred/src/scooping.rec'))
	    #for safe interrupt handling
	    #rospy.on_shutdown(traj.stop)
	    traj.start()
	    result = traj.wait()
	    print("Exiting - File Playback Complete")

	def playback_gripping(self):
	    # print("Getting robot state... ")
	    # rs = baxter_interface.RobotEnable(CHECK_VERSION)
	    # print("Enabling robot... ")
	    # rs.enable()
	    print("Running Replay. Ctrl-c to quit")
	    traj = Trajectory()
	    traj.parse_file(path.expanduser('/home/robin/catkin_ws/src/fred/src/gripping.rec'))
	    #for safe interrupt handling
	    #rospy.on_shutdown(traj.stop)
	    traj.start()
	    result = traj.wait()
	    print("Exiting - File Playback Complete")

if __name__ == '__main__':
	rospy.init_node("fred_node") 
	fred = BaxterControl(arm="right")
	# fred.calibrate('left')
	# fred.open_gripper()



	# print(fred.get_joints())

	fred.playback_gripping()
	# # after scooping
	# x = 0.387996862607
	# y = 0.00424926186825
	# z = 0.675423973195


	# nx = 0.918798574318
	# ny = -0.189954828611
	# nz = 0.493011151821


	## TEST 1 : x and then y
	# current = fred.get_end_effector_pos()
	# print(current)
	# fred.set_end_effector_pos(nx, ny, nz)
	# print(new)
	# fred.set_end_effector_pos(new_x, new_y, current.z)

	## TEST 2 : go closer and then feed (prerec more)

	## TEST 3 :


	
	#fred.playback_scooping()
	# x= 1.1089942325
	# y= -0.369135262722
	# z= 0.474725678011
	# x = 1.13331260849 
	# y = -0.288359962252
	# z = 0.31488774704
	# x = 19.995280359209048
	# y = 1.3563419959876408
	# z = 3.899322989418465
	# fred.get_end_effector_pos()
	# x, y, z = 1.011, 0.122, 0.86
	# x, y, z = 1.18669177922, -0.170612443203, 0.325592013668
	# fred.set_end_effector_pos(x, y, z)
	# fred.set_joint_angles([0.4391019940376282, -0.3773592710494995, -2.3239808082580566, -1.538966178894043, 3.0518548488616943, -0.1859951764345169, -0.049854375422000885])


