#!/usr/bin/env python
# Grace Chin 2018
''' Python Module to control Baxter Robot.

On separate terminals, run the following:
	$ rosrun baxter_interface joint_trajectory_action_server.py --mode velocity
	$ python baxter_pub.py
'''

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
from trac_ik_python.trac_ik import IK # import for inverse_kinematics
import ast
from os import path


class BaxterControl:
	"""Class containing methods to control an instance of the Baxter."""

	def __init__(self, arm="right"):
		self.default_arm = arm #default arm defined
		self.ee_face_forward = [-0.44540951393, 0.551357225282, -0.475534129977, 0.521040177201] #orientation of the end effector when the spoon is feeding

	def get_joints(self, limb=None):
		'''Gets the 3 values that define Baxter's 7 joint angles (in radians) of Baxter's defined limbs'''
		if (limb==None): limb = self.default_arm
		publisher_name = 'baxter_' + limb + '_joints'
		ee_ori_sub = rospy.Subscriber(publisher_name, Float32MultiArray, self.return_msg)
		rospy.wait_for_message(publisher_name, Float32MultiArray)
		return msg

	def get_end_effector_ori(self, limb=None):
		'''Gets the 4 values that define Bxter's end effector orientation'''
		if (limb==None): limb = self.default_arm
		publisher_name = 'baxter_' + limb + '_ori'
		ee_ori_sub = rospy.Subscriber(publisher_name, Quaternion, self.return_msg)
		rospy.wait_for_message(publisher_name, Quaternion)
		return msg

	def get_end_effector_pos(self, limb=None):
		'''Gets the 3 values that define Bxter's end effector position'''
		if (limb==None): limb = self.default_arm
		publisher_name = 'baxter_' + limb + '_pos'
		ee_pos_sub = rospy.Subscriber(publisher_name, Point, self.return_msg)
		rospy.wait_for_message(publisher_name, Point)
		return msg

	def return_msg(self, message):
		''' Callback function of the subscriber methods to return the messages given'''
		global msg
		msg = message
		return msg

	def set_joint_angles(self, joints, limb=None):
		''' joints = [...] <-- list of 7 joint angles. Sets Baxter’s limb to the given joint angles.'''
		print('Moving there')
		if (limb==None): limb = self.default_arm
		arm = baxter_interface.Limb(limb)
		jn = arm.joint_names()
		positions = dict(zip(jn, joints))
		arm.move_to_joint_positions(positions, timeout=15.0, threshold=0.008726646)
		rospy.sleep(4.)

	def calibrate_gripper(self, limb=None):
		'''Calibrates the gripper. Calibration of the gripper must be done before any movement of the gripper.'''
		if (limb==None): limb = self.default_arm
		gripper = baxter_interface.Gripper(limb, CHECK_VERSION)
		gripper.calibrate()

	def close_gripper(self, limb=None):
		'''Close the gripper.'''
		if (limb==None): limb = self.default_arm
		gripper = baxter_interface.Gripper(limb, CHECK_VERSION)
		gripper.close()

	def open_gripper(self, limb=None):
		'''Opens the gripper.'''
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
		ik_solver = IK("base", limb+"_gripper", urdf_string=urdf_str) 
		lb, up = ik_solver.get_joint_limits() 
		# Setting lower and upper limits to the joints
		ik_solver.set_joint_limits([-1.70168, -2.147, -3.05418, -0.05, -3.059, -1.5708, -3.059], 
			[1.70168, 1.047, 3.05418, 2.618, 3.059, 2.094, 3.059])

		seed_state = [0.0] * ik_solver.number_of_joints
		bx = by = bz = 0.02
		brx = bry = brz = 9999.9
		sol = str(ik_solver.get_ik(seed_state, x, y, z, qx, qy, qz, qw, bx, by, bz, brx, bry, brz)) # put these in as integers 
		print('sol', sol)
		sol.replace('(','[');
		sol.replace(')',']');
		sol = ast.literal_eval(sol)
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


