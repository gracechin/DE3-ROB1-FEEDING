from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import message_filters 
from message_filters import Subscriber
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


#dont forget to $ roscore and  $ roslaunch openni2_launch openni2.launch
#https://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/ FOR MULTPLE INPUTS

predictor_path = "/home/robin/DE3-ROB1-FEEDING/perception/shape_predictor_68_face_landmarks.dat"
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)

rospy.init_node('image_converter', anonymous=True)
image_pub = rospy.Publisher("/mouthxy", String, queue_size=10)

bridge = CvBridge()
image_sub = Subscriber("/camera/rgb/image_rect_color",Image,queue_size =1)
depth_sub = Subscriber("/camera/depth_registered/image_raw", Image,queue_size =1)

tss = message_filters.TimeSynchronizer([image_sub, depth_sub],10)

def callback_d(img,depth):
    print("running")
    try:
      depth_image_raw = self.bridge.imgmsg_to_cv2(depth, "passthrough")
      depth_image = (100*depth_image_raw).astype(np.uint8)
    except CvBridgeError as e:
      print(e)
    try:
      cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Depth window", cv_image)
    cv2.imshow("Image window", depth)
                                          
tss.registerCallback(callback_d)
print('init')




rospy.spin()