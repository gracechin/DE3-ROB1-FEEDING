from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2
import message_filters

#dont forget to roslaunch openni2_launch openni2.launch

# can run subscribers independently but not together, the call back works fine
# can 
predictor_path = "/home/robin/DE3-ROB1-FEEDING/perception/shape_predictor_68_face_landmarks.dat"
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)

class image_converter:

  def __init__(self):

		self.bridge = CvBridge()
		#seperate topics:
		self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.callback)
		self.hand_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,self.callback_hand)
		# self.hand_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.callback_hand)

		# Below packaging together topics:
		# # image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color",Image)
		# image_sub = message_filters.Subscriber("/cameras/right_hand_camera/image",Image)
		# hand_sub = message_filters.Subscriber("/cameras/right_hand_camera/image", Image)
		# # hand_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)

		# # packages the topics together because they are being at different frequencies
		# tss = message_filters.ApproximateTimeSynchronizer([image_sub, hand_sub],queue_size=10, slop=1)
		# print('message')
		# tss.registerCallback(self.callback_hand)
		# print('message sent')


  def callback(self,data):
    print("normal callback")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.cv_image = cv_image

    
    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(1)

  def callback_hand(self,data):
    print("hand callback")
    try:
      hand_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.hand_image = hand_image

  def callback_together(self,data, hand):
    print("normal callback")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.cv_image = cv_image


    
    # cv2.imshow("Image_2 window", hand_image)
    # cv2.waitKey(1)
    

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
