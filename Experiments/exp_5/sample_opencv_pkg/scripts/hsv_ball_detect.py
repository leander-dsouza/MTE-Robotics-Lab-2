#!/usr/bin/env python3

import cv2
import numpy as np

cap=cv2.VideoCapture(0)
ret,image=cap.read()
cap.release()
cv2.imshow("image", image)
cv2.waitKey(0) 
cv2.destroyAllWindows()


def mouse(event,x,y,flags,param):
	if event==cv2.EVENT_LBUTTONDOWN:
		h=hsv[y,x,0]
		s=hsv[y,x,1]
		v=hsv[y,x,2]
		print("H:",h)
		print("S:",s)
		print("V:",v)

cv2.namedWindow('mouse')
cv2.setMouseCallback('mouse',mouse)



cap=cv2.VideoCapture(0)
ret,image=cap.read()
cap.release()

cv2.imshow("original image", image)

hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
cv2.imshow("mouse", hsv)



cv2.waitKey(0)
cv2.destroyAllWindows()

redLower=(15,200,150)
#varies based on the color of the ball
redUpper=(30,250,180)


mask=cv2.inRange(hsv,redLower,redUpper)
cv2.imshow("mask",mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

kernel=np.ones((3,3),np.uint8)
mask=cv2.dilate(mask,kernel,iterations=1)
_, contours, hierarchy=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
cv2.imshow("dilate",mask)
cv2.waitKey(0)
cv2.destroyAllWindows()
print("Number of contours found="+str(len(contours)))


def get_contour_center(contour):
	M=cv2.moments(contour)
	cx=-1
	cy=-1
	if(M['m00']!=0):
		cx=int(M['m10']/M['m00'])
		cy=int(M['m01']/M['m00'])
	return cx,cy


for c in contours:
	area = cv2.contourArea(c)
	perimeter = cv2.arcLength(c, True)
	((x,y),radius)= cv2.minEnclosingCircle(c)
	if (area>100):
		cv2.drawContours(image,[c],-1,(150,250,150),2)
		cv2.drawContours(mask,[c],-1,(150,250,150),2)
		cx, cy = get_contour_center(c)
		cv2.circle(image, (cx,cy), (int)(radius), (0,0,255),3)
		#cv2.circle(mask, (cx,cy), (int)(radius), (0,0,255),3) 
		#cv2.circle(mask, (cx,cy), 5, (0,0,255),-1)
		print("Area.{},Perimeter.{}".format(area,perimeter))
		print("number of contours:{}".format(len(contours)))
		cv2.imshow("RGB", image)
		cv2.imshow("BW", mask)

cv2.waitKey(0) 
cv2.destroyAllWindows()