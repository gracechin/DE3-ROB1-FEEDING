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
			cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			print(e)

        # show images
		cv2.imshow("Depth window", depth_image)
		cv2.imshow("Image window", cv_image)
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
