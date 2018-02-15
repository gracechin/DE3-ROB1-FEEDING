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

#dont forget to $ roscore and  $ roslaunch openni2_launch openni2.launch
#https://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/ FOR MULTPLE INPUTS

predictor_path = "/home/robin/DE3-ROB1-FEEDING/shape_predictor_68_face_landmarks.dat"
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/mouthxy", String, queue_size=10)

    self.bridge = CvBridge()
    self.image_depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.callback_d,queue_size =1)
  
  def callback_d(self,data):
    try:
      depth_image_raw = self.bridge.imgmsg_to_cv2(data, "passthrough")

      depth_image = (100*depth_image_raw).astype(np.uint8)

      # depth_image = np.asarray(depth_image_raw)

    except CvBridgeError as e:
      print(e)
    
    print(depth_image[300,300])

    cv2.imshow("Depth window", depth_image_raw)
    cv2.imshow("Depth window2", depth_image)
    cv2.waitKey(1)


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
