#!/usr/bin/env python

import rospy
import rospkg
import baxter_interface
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



class BaxterControl:
	def __init__(self, arm="right"):
		self.fred_node = rospy.init_node("fred_node") 
		self.default_arm = arm
		self.ee_face_forward = [0.6879941859629728, 0.031021910731428202, 0.7249706767167493, -0.010915999110311306]

	def get_joints(self, limb=None): 
		if (limb==None): limb = self.default_arm
		publisher_name = 'baxter_' + limb + '_joints'
		ee_ori_sub = rospy.Subscriber(publisher_name, Float32MultiArray, self.return_msg)
		rospy.wait_for_message(publisher_name, Float32MultiArray)
		return msg

	def get_ee_ori(self, limb=None): 
		if (limb==None): limb = self.default_arm
		publisher_name = 'baxter_' + limb + '_ori'
		ee_ori_sub = rospy.Subscriber(publisher_name, Quaternion, self.return_msg)
		rospy.wait_for_message(publisher_name, Quaternion)
		return msg

	def get_ee_pos(self, limb=None): 
		if (limb==None): limb = self.default_arm
		publisher_name = 'baxter_' + limb + '_pos'
		ee_pos_sub = rospy.Subscriber(publisher_name, Point, self.return_msg)
		rospy.wait_for_message(publisher_name, Point)
		return msg

	def return_msg(self, message):
		global msg
		msg = message
		return msg

	def get_end_effector_pos(self, limb=None):
		'''Gets joint angles of Baxter, limb = 'left or right'
		Return = dict of end effector position'''
		if (limb==None): limb = self.default_arm
		arm = baxter_interface.Limb(limb)
		end_pos = arm.endpoint_pose()
		print('End effector pos', end_pos)
		return end_pos

	def set_joint_angles(self, joints, limb=None):
		'''Sets joint angles of Baxter 
		joints = list of the desired joint angles
		Baxter moves joints to the angles'''
		if (limb==None): limb = self.default_arm
		arm = baxter_interface.Limb(limb)
	 	jn = arm.joint_names()
		print("before joint angles:", arm.joint_angles())
		positions = dict(zip(jn, joints))
		print("desired joint angles:", positions)
		arm.move_to_joint_positions(positions, timeout=15.0, threshold=0.008726646)
		print("after joint angles:", arm.joint_angles())
		rospy.sleep(4.)

	def set_end_effector_pos(self, x, y, z, qx=None, qy=None, qz=None, qw=None, limb=None):
		'''Sets end effector position of Baxter
		x, y, z = positions of the end effector
		qx, qy, qz, qw = orientation of the end effector (Quaternion)
		Change in Baxter end effector'''
		if (qx == None): qx, qy, qz, qw = self.ee_face_forward[0], self.ee_face_forward[1], self.ee_face_forward[2], self.ee_face_forward[3]    
		if (limb==None): limb = self.default_arm
		urdf_str = rospy.get_param('/robot_description')
		# Defining the two frames of reference wanted on the robot
		ik_solver = IK("base", limb+"_gripper", urdf_string=urdf_str)
		lb, up = ik_solver.get_joint_limits() 
		# Setting lower and upper limits to the joints
		ik_solver.set_joint_limits([-1.70168, -2.147, -3.05418, -0.05, -3.059, -1.5708, -3.059], 
			[1.70168, 1.047, 3.05418, 2.618, 3.059, 2.094, 3.059])
		seed_state = [0.0] * ik_solver.number_of_joints
		# Inserting desired points for the solver to solve
		sol = str(ik_solver.get_ik(seed_state, x, y, z, qx, qy, qz, qw)) # put these in as integers 
		# converting sol from a string to a list
		sol.replace('(','[');
		sol.replace(')',']');
		sol = ast.literal_eval(sol)
		print('Desired position:', [x, y, z])
		print('Desired joint Angles:', sol)
		print('setting joint angles....')
		self.set_joint_angles(sol)
		return sol




if __name__ == '__main__':
	fred = BaxterControl(arm="right")
	# fred.get_joint_angles()
	pos = fred.get_ee_pos()
	ori = fred.get_ee_ori()
	joi = fred.get_joints()
	print(pos.x, pos.y, pos.z)
	print(ori.x, ori.y, ori.z, ori.w)
	print(joi.data)
	# pos = eep['position']
	# # ori = eep['orientation']
	# print(type(eep['position']))
	# x, y, z, Qx, Qy, Qz, Qw =pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w
	# print(x, y, z, Qx, Qy, Qz, Qw)
	# fred.set_end_effector_pos(1.054203949032971, -0.5024091495059471, 0.5046310309715524)


