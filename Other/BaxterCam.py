from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import message_filters
from message_filters import Subscriber
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2


class Mouth:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.callback)

	def callback(self,data):
		try:
		  cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
		  print(e)
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(1)


def main(args):
  ic = Mouth()
  rospy.init_node('Mouth', anonymous=True)
  try:
	rospy.spin()
  except KeyboardInterrupt:
	print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
