from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2

predictor_path = "/home/robin/catkin_ws/src/fred/src/perception/shape_predictor_68_face_landmarks.dat"
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)

class image_converter:

	def __init__(self):
		self.candy_pub = rospy.Publisher("/candy",String, queue_size=10)
		self.face_status_pub = rospy.Publisher("/face_status", String, queue_size=10)
		self.mouth_xy_pub = rospy.Publisher("/mouth_xy", Point, queue_size=10)
		self.mouth_status_pub = rospy.Publisher("/mouth_status", String, queue_size=10)
		self.candycount = 0

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,self.callback)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			cv_image = imutils.rotate(cv_image, 90)
		except CvBridgeError as e:
			print(e)

		spoon_ROI = cv_image[225:255,225:290]
		self.food(spoon_ROI)
		self.mouth(cv_image)

		
	def mouth(self,cv_image):
		
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		rects = detector(gray, 1)
		if len(rects) > 0:
			face_status = 'True'
		else: 
			face_status = 'False'


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
			mouth_left = shape[61]
			mouth_right = shape[65]

			mouth_center_x = mouth_bottom[0] +(mouth_top[0]-mouth_bottom[0])/2
			mouth_center_y = mouth_bottom[1] +(mouth_top[1]-mouth_bottom[1])/2
			cv2.circle(cv_image, (mouth_center_x, mouth_center_y), 1, (255, 0, 255), 5)

			xyz = Point()
			xyz.x = mouth_center_x
			xyz.y = mouth_center_y
			xyz.z = 0

			mouth_ratio = float(abs(mouth_top[1]- mouth_bottom[1])/(abs(mouth_left[0] - mouth_right[0])))

			if mouth_ratio > 0: 
				mouth_status = 'True'
			else: 
				mouth_status = 'False'
			self.mouth_xy_pub.publish(xyz)		
			self.mouth_status_pub.publish(mouth_status)
			print('xyz', xyz)
			print('mouth_status', mouth_status)


		self.face_status_pub.publish(face_status)
		print('face_status', face_status)
		
		cv2.imshow("Image window", cv_image)
		
		cv2.waitKey(1)

	def food(self,frame):
		candy_state = 'False'
		candy_trigger = 'False'
		threshold_area = 0
		blurred = cv2.GaussianBlur(frame, (5, 5), 0)

		# convert colours to hsv
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	 
		# define range of green color in HSV
		lower_green = np.array([45, 50, 20])
		upper_green = np.array([75, 255, 255])
 
		# threshold the HSV image to get only green colors
		mask = cv2.inRange(hsv, lower_green, upper_green)
 		res = cv2.bitwise_and(frame,frame,mask=mask)
 
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
						self.candycount +=1
						candy_trigger = 'True'

		if self.candycount >= 5:
			candy_state = 'True'
			print('counted 5')

		if candy_trigger == 'False':
			self.candycount = 0 
			candy_state = 'False'
		
		print ('candy state', candy_state)

		self.candy_pub.publish(candy_state)					
		cv2.imshow("Food window", frame)
		cv2.imshow("Mask window", mask)
		cv2.imshow("Res Window", res)

		


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

