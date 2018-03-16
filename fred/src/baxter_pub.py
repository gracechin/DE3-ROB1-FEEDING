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



class BaxterPub:
	def __init__(self):
		self.right_pos = rospy.Publisher("baxter_right_pos", Point, queue_size=10)
		self.left_pos = rospy.Publisher("baxter_left_pos", Point, queue_size=10)
		self.right_ori = rospy.Publisher("baxter_right_ori", Quaternion, queue_size=10)
		self.left_ori = rospy.Publisher("baxter_left_ori", Quaternion, queue_size=10)
		self.right_joints = rospy.Publisher("baxter_right_joints", Float32MultiArray, queue_size=10)
		self.left_joints = rospy.Publisher("baxter_left_joints", Float32MultiArray, queue_size=10)

	def endeffector_info(self, limb): 
		arm = baxter_interface.Limb(limb)
		end_effector = arm.endpoint_pose()
		end_pos = end_effector['position']
		end_ori = end_effector['orientation']
		pos = Point()
		ori = Quaternion()
		pos.x, pos.y, pos.z = end_pos.x, end_pos.y, end_pos.z
		ori.x, ori.y, ori.z, ori.w = end_ori.x, end_ori.y, end_ori.z, end_ori.w
		return pos, ori

	def joints_info(self, limb):
		arm = baxter_interface.Limb(limb)
		angles = arm.joint_angles().values()
		joints = Float32MultiArray()
		joints.data = angles
		return joints

if __name__ == '__main__':
	rospy.init_node("baxter_pub") 
	rate = rospy.Rate(10) #10Hz

	while True:
		baxter_pub = BaxterPub()

		right_pos, right_ori = baxter_pub.endeffector_info("right")
		left_pos, left_ori = baxter_pub.endeffector_info("left")
		right_angles = baxter_pub.joints_info("right")
		left_angles = baxter_pub.joints_info("left")
		print('right_pos', right_pos)
		baxter_pub.right_pos.publish(right_pos)
		baxter_pub.right_ori.publish(right_ori)

		baxter_pub.left_pos.publish(left_pos)
		baxter_pub.left_ori.publish(left_ori)

		baxter_pub.right_joints.publish(right_angles)
		baxter_pub.left_joints.publish(left_angles)

		rate.sleep()





