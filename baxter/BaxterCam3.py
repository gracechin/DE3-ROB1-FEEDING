from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2


### i can access baxter topics and i can acess oppenni topics but not at the same time. baxterbash means i cant connect to astra

predictor_path = "/home/robin/DE3-ROB1-FEEDING/perception/shape_predictor_68_face_landmarks.dat"
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)

class image_converter:

	def __init__(self):
		#self.image_pub = rospy.Publisher("/camera/rgb/image_rect_color",Image)

		self.bridge = CvBridge()

		# hand_image_sub = message_filters.Subscriber("/cameras/right_hand_camera/image",Image)
		hand_image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color",Image)
		image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color",Image)
		# depth_sub = message_filters.Subscriber("/camera/depth_registered/image_raw", Image)
	
		# tss = message_filters.ApproximateTimeSynchronizer([hand_image_sub, image_sub],queue_size=10, slop=0.5)	
		# tss.registerCallback(self.callback)
		print('init done')
	def callback(self,hand,img):
		print('callback')
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			print(e)
		# try:
		# 	depth_image_raw = self.bridge.imgmsg_to_cv2(depth, "passthrough")
		# 	depth_image = ((255*depth_image_raw)).astype(np.uint8)
		# except CvBridgeError as e:
		# 	print(e)
		try:
			hand_image = self.bridge.imgmsg_to_cv2(hand, "bgr8")
			hand_image = imutils.rotate(cv_image, 90)
		except CvBridgeError as e:
			print(e)

		spoon_ROI = cv_image[220:280,190:280]
		self.food(spoon_ROI)
		self.mouth(cv_image)
		self.mouth(hand_image)

		
	def mouth(self,cv_image):
		
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		rects = detector(gray, 1)

		for (i, rect) in enumerate(rects):
			shape = predictor(gray, rect)
			shape = face_utils.shape_to_np(shape)
			features = face_utils.FACIAL_LANDMARKS_IDXS.items()
			mouth = features[0]
			points = shape[mouth[1][0]:mouth[1][1]]
			for (x,y) in points: 
				cv2.circle(cv_image, (x, y), 1, (0, 0, 255), -1)

			inside_points = shape[60:68]
			mouth_top = shape[62]
			mouth_bottom = shape[66]
			mouth_center_x = mouth_bottom[0] +(mouth_top[0]-mouth_bottom[0])/2
			mouth_center_y = mouth_bottom[1] +(mouth_top[1]-mouth_bottom[1])/2
			cv2.circle(cv_image, (mouth_center_x, mouth_center_y), 1, (255, 0, 255), 5)
			print (mouth_center_x, mouth_center_y)

		cv2.imshow("Image window", cv_image)
		
		cv2.waitKey(1)
	def food(self,frame):
		
		threshold_area = 0
		blurred = cv2.GaussianBlur(frame, (5, 5), 0)

		# convert colours to hsv
		#hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	 
		# define range of green color in HSV
		lower_green = np.array([30, 144, 113])
		upper_green = np.array([120, 150, 127])
 
		# threshold the HSV image to get only green colors
		mask = cv2.inRange(blurred, lower_green, upper_green)
 
 
		# find contours in the thresholded image
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0] if imutils.is_cv2() else cnts[1]
 
		# loop over the contours
		for c in cnts:
				area = cv2.contourArea(c)
 
				if area > threshold_area:
						M = cv2.moments(c)
 
						# to not divide by zero
						if (M["m00"] == 0): M["m00"] = 1
 
						cX = int(M['m10'] / M['m00'])
						cY = int(M['m01'] / M['m00'])
 
						# rows, cols = mask.shape[:2]
						# [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
						# lefty = int((-x * vy / vx) + y)
						# righty = int(((cols - x) * vy / vx) + y)
 
						# cv2.line(frame,(cols-1,righty),(0, lefty),(0, 255,0),2)
						#cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
						cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
						cv2.putText(frame, "candy", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
		cv2.imshow("Food window", frame)
		cv2.imshow("Mask window", mask)

		


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

