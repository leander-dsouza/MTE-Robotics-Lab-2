#!/usr/bin/env python3
from __future__ import print_function
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
import sys
import numpy as np

bridge=CvBridge()

def rgb_image_callback(ros_image):
	print('got an image')
	global bridge
	try:
		rgb_image=bridge.imgmsg_to_cv2(ros_image,"bgr8")
	except CvBridgeError as e:
		print(e)

	rgb_array=np.array(rgb_image,dtype=np.uint8)
	edge=process_image(rgb_array)	
	cv2.imshow("RGB Image window",rgb_array)
	cv2.imshow("Edges detected",edge)
	cv2.waitKey(3)

def depth_image_callback(ros_image):
	print('got an image')
	global bridge
	try:
		depth_image=bridge.imgmsg_to_cv2(ros_image,"32FC1")
	except CvBridgeError as e:
		print(e)

	depth_array=np.array(depth_image,dtype=np.float32)	
	cv2.normalize(depth_array,depth_array,0,1,cv2.NORM_MINMAX)
	cv2.imshow("Depth Image window",depth_array)
	cv2.waitKey(3)

def process_image(RGB_image):
	grey=cv2.cvtColor(RGB_image, cv2.COLOR_BGR2GRAY)
	grey=cv2.blur(grey,(7,7))
	edges=cv2.Canny(grey,15.0,30.0)
	return edges

def main(args):
	rospy.init_node('image_converter',anonymous=True)
	image_depth_sub=rospy.Subscriber("/camera/depth/image_raw", Image,depth_image_callback)
	image_rgb_sub=rospy.Subscriber("/camera/color/image_rect_raw", Image,rgb_image_callback)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)