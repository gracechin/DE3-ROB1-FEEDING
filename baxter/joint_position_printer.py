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
)

def main():
	rospy.init_node("joint_position_printer")

	left = baxter_interface.Limb('left')
	current_position = left.joint_angles()
	print current_position
	end_pos = left.endpoint_pose()
	print end_pos

"""
For checking if my IK works, I need to find the IK solution from one point to another. here is the elbow point:
x=0.01576723288557902, y=0.6516476157913255, z=0.4534633791783551

here is the end effector point:
x=0.5889929892125815, y=0.7231788386129996, z=0.33973628833773745
"""

if __name__ == '__main__':
	main()
