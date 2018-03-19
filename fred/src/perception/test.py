from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2

cap = cv2.VideoCapture(0)

while(True):
	ret,frame = cap.read()
	print type(frame)
	
	cv2.imshow('frame', frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
