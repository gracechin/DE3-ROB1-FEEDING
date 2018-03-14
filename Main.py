import rospy
from std_msgs.msg import String
# import message_filters
# from message_filters import Subscriber
from geometry_msgs.msg import Point
import sys
import cv2


# this is the main code that executes operations and acquires data from other files
# before running main make sure you have all the other files running

class Run:
	def __init__(self):
		## input from Astra
		self.face_status_astra = rospy.Subscriber("/face_status_astra", String, self.callback)
		
		# self.mouth_status_astra = Subscriber("/mouth_status_astra", String)

		## input from Baxter
		#self.candy_status = rospy.Subscriber("/candy", String, self.callback)
		# self.face_status_baxter = Subscriber("/mouth_status_astra", String)
		# self.mouth_xy_baxter = Subscriber("/mouth_xy", Point)
		# self.mouth_status_baxter = Subscriber("/mouth_status", String)
		# print(self.candy_status)


		

	def callback(self, C):
		# print(C)
		if C.data == 'True':
			self.mouth_xyz_astra = rospy.Subscriber("/mouth_xyz_astra", Point, self.mouth)
		else: 
			print(C)
	def mouth(self, M):
		print(M)
		

	# def pickupfood(self):

	# def gotomouth(self):
	# 	pass

	# def release(self):
class Run2:
	def __init__(self):
		self.val = 12

	def function(self, data):
		print (data)

def main(args):
	ic = Run()
	rospy.init_node('Run', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	#cv2.destroyAllWindows()

if __name__ == '__main__':
		main(sys.argv)

