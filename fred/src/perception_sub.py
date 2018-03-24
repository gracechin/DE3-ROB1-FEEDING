# !/usr/bin/env python
''' Python Module to subscribe to different camera topics.

On separate terminals, run the following:
	$ roslaunch openni2_launch openni2.launch
	$ python cam_kinetic.py
	$ cam_baxter.py
'''
import rospy
import rospkg
# from geometry_msgs.msg import (PoseStamped,Pose,Point,Quaternion)
# from std_msgs.msg import (Header,Empty)
from message_filters import Subscriber
import message_filters
from baxter_control import BaxterControl
from geometry_msgs.msg import Point
from std_msgs.msg import String

class PerceptionSub:
    ''' Class to subscribe to get different perception information'''
    def __init__(self):
        self.baxter_mouth_pos = None
        self.kinect_mouth_pos = None
        self.baxter_mouth_status = None
        self.kinect_mouth_status = None
        self.baxter_fae_status = None
        self.kinect_face_status = None
        self.baxter_mouth_status = None
        self.baxter_candy_status = None

    def get_baxter_mouth_pos(self):
        ''' Gets mouth position from baxter's hand camera
        Detects mouth position
        returns Point'''

        def update(msg): 
            self.baxter_mouth_pos = msg.data
        rospy.Subscriber("/mouth_xyz", Point, self.return_point)
        rospy.wait_for_message("/mouth_xyz", Point)

    def get_kinect_mouth_pos(self):
        ''' Gets mouth position from the kinetic camera
        Detects mouth position
        returns Point'''

        def update(msg): 
            self.kinect_mouth_pos = msg.data
        rospy.Subscriber("/mouth_xyz_kinetic", Point, update)
        rospy.wait_for_message("/mouth_xyz_kinetic", Point)

    def get_baxter_mouth_status(self):
        ''' Gets mouth status from baxter's hand camera
        Detects mouth open or close
        returns True or False'''

        def update(msg): 
            if msg.data=="True": ft = True
            else: ft = False
            self.baxter_mouth_status = ft
        rospy.Subscriber("/mouth_status", String, update)
        rospy.wait_for_message("/mouth_status", String)

    def get_kinect_mouth_status(self):
        ''' Gets mouth status from the kinetic camera'''
        def update(msg): 
            if msg.data=="True": ft = True
            else: ft = False
            self.kinect_mouth_status = ft
        rospy.Subscriber("/mouth_status_kinetic", String, self.update)
        rospy.wait_for_message("/mouth_status_kinetic", String)

    def get_baxter_face_status(self):
        ''' Gets mouth status from the baxter hand camera
        Detects mouth open or close
        returns True or False'''

        def update(msg): 
            if msg.data=="True": ft = True
            else: ft = False
            self.baxter_face_status = ft
        rospy.Subscriber("/face_status", String, update)
        rospy.wait_for_message("/face_status", String)

    def get_kinect_face_status(self):
        ''' Gets face status from the kinetic camera
        Detects face presence
        returns True or False'''

        def update(msg): 
            if msg.data=="True": ft = True
            else: ft = False
            self.kinect_face_status = ft
        rospy.Subscriber("/face_status_kinetic", String, update)
        rospy.wait_for_message("/face_status_kinetic", String)

    def get_baxter_candy_status(self):
        ''' Gets candy status from the baxter hand camera
        Detects Candy presence
        return True or False'''

        def update(msg): 
            if msg.data=="True": ft = True
            else: ft = False
            self.kinect_candy_status = ft
        rospy.Subscriber("/candy", String, update)
        rospy.wait_for_message("/candy", String)


if __name__ == '__main__':
    rospy.init_node("perception_sub_node")
    perceptioninfo = PerceptionSub()

