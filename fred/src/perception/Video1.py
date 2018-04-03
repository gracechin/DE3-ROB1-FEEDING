from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2

#from primesense import openni2
#from primesense import _openni2 as c_api
#openni2.initialize("<PATH TO OPENNI2 REDIST FOLDER>")


cap = cv2.VideoCapture(0)
predictor_path = "/home/robin/Documents/Feeding_Robot/shape_predictor_68_face_landmarks.dat"

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)

while(True):
	ret,frame = cap.read()
	print type(frame)
	image = frame
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	clone = frame
	rects = detector(gray, 1)

	for (i, rect) in enumerate(rects):
		shape = predictor(gray, rect)
		shape = face_utils.shape_to_np(shape)
		features = face_utils.FACIAL_LANDMARKS_IDXS.items()
		mouth = features[0]
		points = shape[mouth[1][0]:mouth[1][1]]
		for (x,y) in points: 
			cv2.circle(clone, (x, y), 1, (0, 0, 255), -1)

		inside_points = shape[60:68]
		mouth_top = shape[62]
		mouth_bottom = shape[66]
		mouth_center_x = mouth_bottom[0] +(mouth_top[0]-mouth_bottom[0])/2
		mouth_center_y = mouth_bottom[1] +(mouth_top[1]-mouth_bottom[1])/2
		cv2.circle(clone, (mouth_center_x, mouth_center_y), 1, (255, 0, 255), 5)
	
		cv2.polylines(clone, [inside_points],True, (0,255,255))
	
	cv2.imshow('frame', frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

# COLOR CHANGING METHODS:
# cv2.COLOR_BAYER_BG2BGR
# cv2.COLOR_BGR2GRAY
# cv2.COLOR_BGR2HSV
