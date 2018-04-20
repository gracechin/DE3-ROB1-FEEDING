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
import numpy as np
import argparse
import cv2
import imutils

#NOTE:
##### Need to install Openni, for ASTRA follow instructions here http://jsk-recognition.readthedocs.io/en/latest/install_astra_camera.html
##### Before running this code run:
########## $ roscore and  $ roslaunch openni2_launch openni2.launch

class image_converter:

	def __init__(self):
				# set up publisher if required
		self.image_pub = rospy.Publisher("/mouthxy", String, queue_size=10)

				# sets up cv bridge which converts the subscribed topic into a cv image
		self.bridge = CvBridge()

				# subscribes to the rgb and depth topics
		image_sub = Subscriber("/camera/rgb/image_rect_color",Image)
		depth_sub = Subscriber("/camera/depth_registered/image_raw", Image)

				# packages the topics together because they are being at different frequencies
		tss = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub],queue_size=10, slop=0.5)

				# call the callback function with these subscribed topics
		tss.registerCallback(self.callback)
		print('init')

	def callback(self,img,depth):
				# convert the subscrbed topics to cv images using cv bridge
		try:
			depth_image_raw = self.bridge.imgmsg_to_cv2(depth, "passthrough")
						#convert the depth image into a numpy array with values between 0 and 255. you can now access these depth values
			depth_image = ((255*depth_image_raw)).astype(np.uint8)
		except CvBridgeError as e:
			print(e)
		try:
			frame = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			print(e)
			
		threshold_area = 500
		blurred = cv2.GaussianBlur(frame, (5, 5), 0)

		# convert colours to hsv
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	 
		# define range of green color in HSV
		lower_green = np.array([40, 100, 100])
		upper_green = np.array([70, 255, 255])
 
		# threshold the HSV image to get only green colors
		mask = cv2.inRange(hsv, lower_green, upper_green)
 
 
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
 
						rows, cols = mask.shape[:2]
						[vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
						lefty = int((-x * vy / vx) + y)
						righty = int(((cols - x) * vy / vx) + y)
 
						cv2.line(frame,(cols-1,righty),(0, lefty),(0, 255,0),2)
						cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
						cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
						cv2.putText(frame, "candy", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
 

















				# show images
		cv2.imshow("Depth window", depth_image)
		cv2.imshow("Image window", frame)
		cv2.waitKey(1)

				# this is the code to publish a topic:
		# try:
		#   self.image_pub.publish(String(mouthxyz))
		# except CvBridgeError as e:
		#   print(e)

				# check size of images:
		# print(np.size(cv_image) , np.size(depth_image))

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