# Jacob Mitchell 2018
''' Python Module to access and analyse video stream from Kinect camera.

Before running this file, run the following:
	$ cd catkin ws/
	$ baxter bash.sh  	OR        $ roscore
	$ roslaunch openni2_launch openni2.launch
'''

from __future__ import print_function

import sys
import rospy # rospy is the ros python wrapper
import message_filters # Message filters are used to send both RGB and D messages simultaneously
from message_filters import Subscriber
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import dlib # dlib is a machine learning package with built in face and feature detection. check this link for instructions on how to install https://www.pyimagesearch.com/2017/04/03/facial-landmarks-dlib-opencv-python/
from imutils import face_utils # this package contains lots of little short cuts for working with the facial landmarks
import numpy as np
import imutils
import cv2 # opencv package


# define the predictors and detectors
predictor_path = "/home/robin/DE3-ROB1-FEEDING/perception/shape_predictor_68_face_landmarks.dat"
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)


class Mouth:
	'''This is a class that gets run continuously by rospy.spin.
		It is used to determine the location of the mouth and it's state'''
	def __init__(self):
		# initialise the topics being pubished:
		self.face_status_pub = rospy.Publisher("/face_status_kinetic", String, queue_size=10)
		self.mouth_xyz_pub = rospy.Publisher("/mouth_xyz_kinetic", Point, queue_size=10)
		self.mouth_status_pub = rospy.Publisher("/mouth_status_kinetic", String, queue_size=10)

		# initialise bridge:
		self.bridge = CvBridge()

		# initialise subscribers:
		image_sub = Subscriber("/camera/rgb/image_rect_color",Image)
		depth_sub = Subscriber("/camera/depth_registered/image_raw", Image)

		# Because we want to access both the  RGB and Depth images in the same function we need to bundle them togetherw with approx time synch
		tss = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub],queue_size=10, slop=0.5)		# slop allows frames with different time stamps to get packed together
		tss.registerCallback(self.callback) # hit up the callback function with both images
		print('init')

	def callback(self,img,depth):

		try:
			depth_image_raw = self.bridge.imgmsg_to_cv2(depth, "passthrough")
			depth_image = ((255*depth_image_raw)).astype(np.uint8) # this multiplication by 255 is to scale the current 0-1 array to 0-255
		except CvBridgeError as e:
			print(e)
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			print(e)

		#resizing image speeds it up!
		# cloning to make changes without altering original copy. Depth values depend on colour so drawing all over it is dumb af
		cv_image = imutils.resize(cv_image, width=500)
		depth_image = imutils.resize(depth_image, width=500)
		depth_image_clone = depth_image
		cv_image_clone = cv_image

		#converting the image to grayscale 
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		# detect faces
		rects = detector(gray, 1) # detector returns a list of rectangles that contain faces within the image
		# establish if any faces present 
		if len(rects) > 0: # if no rectangles found then we can say that no faces were found in the image
			face_status = 'True'
		else: 
			face_status = 'False'

		print(face_status)

		self.face_status_pub.publish(face_status)

		# #are picture frames different size? both are 480,640 
		# # print(depth_image_clone.shape)

		#for each face identify landmarks 
		for (i, rect) in enumerate(rects): # within the rectangles with faces
			shape = predictor(gray, rect) # run feature detection using the dlib predictor
			shape = face_utils.shape_to_np(shape) 	# shape and features (line below) make it easier to access and index the features
			features = face_utils.FACIAL_LANDMARKS_IDXS.items()
			mouth = features[0]
			points = shape[mouth[1][0]:mouth[1][1]]

			# print mouth points:
			# for (x,y) in points:
			#   cv2.circle(cv_image, (x, y), 1, (0, 0, 255), -1) # draw a circle around each mouth landmark

			# inside_points = shape[60:68] # take a look at the facial landmark numbering image in the perception folder to make sense of the numbers here
			mouth_top = shape[62]
			mouth_bottom = shape[66]
			mouth_left = shape[61]
			mouth_right = shape[65]

			# geometrically finding the center of the mouth:
			mouth_center_x = mouth_bottom[0] +(mouth_top[0]-mouth_bottom[0])/2
			mouth_center_y = mouth_bottom[1] +(mouth_top[1]-mouth_bottom[1])/2

			# open close mouth ratio
			mouth_ratio = float(abs(mouth_top[1]- mouth_bottom[1])/(abs(mouth_left[0] - mouth_right[0])))

			if mouth_ratio > 0: 
				mouth_status = 'open'
			else: 
				mouth_status = 'closed'
		
			# draw circle in mouth center:
			#cv2.circle(cv_image, (mouth_center_x, mouth_center_y), 1, (255, 0, 255), 5)

			# Overlay the mouth center point onto the depth image and access depth value from image

			try:
				mouth_center_z = depth_image[mouth_center_x-55, mouth_center_y+20] # offset is applied due to distance between depth and RGB sensors on kinect
			except:
				mouth_center_z = 0

			## sometimes coordinates fall outside of range of frame, and an error is called. simple if statement could fix this
			mouthxyz = str(mouth_center_x) + " " +str(mouth_center_y) + " " +str(mouth_center_z)
			# draw a circle on mouth center or depth image clone
			cv2.circle(depth_image_clone, (mouth_center_x-55, mouth_center_y+20), 1, (255, 0, 0), 5)

		 	print ("mouth %s x, y, z = " %(i) ,mouthxyz, 'status = ', mouth_status)

			# create a ros point with x, y, z
		 	xyz = Point()
			xyz.x = mouth_center_z
			xyz.y = mouth_center_x
			xyz.z = mouth_center_y

			# publish point pixel coordinates and mouth status
			self.mouth_xyz_pub.publish(xyz)
			self.mouth_status_pub.publish(mouth_status)

		# show images 
		cv2.imshow("Depth window", depth_image_clone)
		cv2.imshow("Image window", cv_image_clone)
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
	print("here")
	main(sys.argv)