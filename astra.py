from __future__ import print_function
import sys
import rospy
import dlib
import roslib
import cv2
import numpy as np
from imutils import face_utils
import message_filters
from message_filters import Subscriber
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


# $ roscore and  $ roslaunch openni2_launch openni2.launch
#https://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/ FOR MULTPLE INPUTS

predictor_path = "/home/robin/DE3-ROB1-FEEDING/perception/shape_predictor_68_face_landmarks.dat"
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)

class MouthPos:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = Subscriber("/camera/rgb/image_rect_color",Image)
		self.depth_sub = Subscriber("/camera/depth_registered/image_raw", Image)
		self.image_pub = rospy.Publisher("mouthxyz", Point, queue_size=10)

		tss = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub],queue_size=10, slop=0.5)															
		tss.registerCallback(self.callback)


	def callback(self,img,depth):

		try:
			depth_image_raw = self.bridge.imgmsg_to_cv2(depth, "passthrough")
			depth_image = ((255*depth_image_raw)).astype(np.uint8)
		except CvBridgeError as e:
			print(e)
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			print(e)
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		rects = detector(gray, 1)
		depth_image_clone = depth_image


		for (i, rect) in enumerate(rects):
			shape = predictor(gray, rect) #react = rectangle
			shape = face_utils.shape_to_np(shape)
			features = face_utils.FACIAL_LANDMARKS_IDXS.items()

			# working out mouth positions
			mouth_top = shape[62]
			mouth_bottom = shape[66]
			mouth_center_x = mouth_bottom[0] +(mouth_top[0]-mouth_bottom[0])/2
			mouth_center_y = mouth_bottom[1] +(mouth_top[1]-mouth_bottom[1])/2
			mouth_center_z = depth_image[mouth_center_x, mouth_center_y]
			mouthxyz = str(mouth_center_x) + " " +str(mouth_center_y) + " " +str(mouth_center_z)
			print(mouthxyz)

			msg = Point()
			msg.x = mouth_center_x
			msg.y = mouth_center_y
			msg.z = mouth_center_z
			self.image_pub.publish(msg)

if __name__ == '__main__':
	MouthPos()
	rospy.init_node('MouthPos', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
