# !/usr/bin/env python

import rospy
import rospkg
# from geometry_msgs.msg import (PoseStamped,Pose,Point,Quaternion)
# from std_msgs.msg import (Header,Empty)
from message_filters import Subscriber
import message_filters
from baxter_control import BaxterControl
from geometry_msgs.msg import Point
from std_msgs.msg import String


# this is the main code that executes operations and acquires data from other files
# before running main make sure you have all the other files running

# roslaunch openni2_launch openni2.launch
# cam_kinetic.py
# cam_baxter.py

class PerceptionSub:
    def __init__(self):
        self.baxter_mouth_pos = None
        self.astra_mouth_pos = None
        self.baxter_mouth_status = None
        self.astra_mouth_status = None
        self.baxter_fae_status = None
        self.astra_face_status = None
        self.baxter_mouth_status = None
        self.baxter_candy_status = None

    def get_baxter_mouth_pos(self):
        def update(msg): 
            self.baxter_mouth_pos = msg.data
        rospy.Subscriber("/mouth_xyz", Point, self.return_point)
        rospy.wait_for_message("/mouth_xyz", Point)

    def get_astra_mouth_pos(self):
        def update(msg): 
            self.astra_mouth_pos = msg.data
        rospy.Subscriber("/mouth_xyz_astra", Point, update)
        rospy.wait_for_message("/mouth_xyz_astra", Point)

    def get_baxter_mouth_status(self):
        def update(msg): 
            if msg.data=="True": ft = True
            else: ft = False
            self.baxter_mouth_status = ft
        rospy.Subscriber("/mouth_status", String, update)
        rospy.wait_for_message("/mouth_status", String)

    def get_astra_mouth_status(self):
        def update(msg): 
            if msg.data=="True": ft = True
            else: ft = False
            self.astra_mouth_status = ft
        rospy.Subscriber("/mouth_status_astra", String, self.update)
        rospy.wait_for_message("/mouth_status_astra", String)

    def get_baxter_face_status(self):
        def update(msg): 
            if msg.data=="True": ft = True
            else: ft = False
            self.baxter_face_status = ft
        rospy.Subscriber("/face_status", String, update)
        rospy.wait_for_message("/face_status", String)

    def get_astra_face_status(self):
        def update(msg): 
            if msg.data=="True": ft = True
            else: ft = False
            self.astra_face_status = ft
        rospy.Subscriber("/face_status_astra", String, update)
        rospy.wait_for_message("/face_status_astra", String)

    def get_baxter_candy_status(self):
        def update(msg): 
            if msg.data=="True": ft = True
            else: ft = False
            self.baxter_candy_status = ft
        rospy.Subscriber("/candy", String, update)
        rospy.wait_for_message("/candy", String)


if __name__ == '__main__':
    rospy.init_node("perception_sub_node")
    perceptioninfo = PerceptionSub()
    perceptioninfo.get_baxter_candy_status()
    print(perceptioninfo.baxter_candy_status)
	# goto start position
	# pick up food
	# detect mouth open
	# go towards mouth
