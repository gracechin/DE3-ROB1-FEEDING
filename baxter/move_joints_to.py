#!/usr/bin/env python

import sys

import rospy
import baxter_interface

from baxter_interface import CHECK_VERSION


def main():
	rospy.init_node("move_joints_to")
	left = baxter_interface.Limb('left')
	left.move_to_neutral(timeout=15.0)
 	lj = left.joint_names()
	print lj
	current_position = left.joint_angles()
	print current_position
	positions = dict(zip(left.joint_names(), [1.5942346723496317, -0.5205685917644269, -1.2826969264221848, 2.364335742017643, 1.9859123276093544, -0.590645109308224, -2.1038481756585465]))
	# joint angles order s0, s1, e0, e1, w0, w1, w2

	left.move_to_joint_positions(positions, timeout=15.0, threshold=0.008726646)
	current_position = left.joint_angles()
	print current_position
	rospy.sleep(4.)
	#left.move_to_neutral(timeout=15.0)
	#current_position = left.joint_angles()
	#print current_position
	

if __name__ == '__main__':
	main()

	
	
### RE INSTALL PYKDL, THEN RE CLONE THE REDONE GITHUB REPOSITORY, AND TRY PYKDL AGAIN. REMEMBER TO SEED THE ANGLES INTO THE INVERSE KINEMATICS SOLVER, WHICH WILL BE CALCULATED DIRECTLY FROM THE VIVE. 
 
