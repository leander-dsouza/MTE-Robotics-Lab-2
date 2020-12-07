#!/usr/bin/env python3

import rospy
from test_pub_sub.msg import test_custom_msg

def Publisher():

	pub=rospy.Publisher('string_publish', test_custom_msg, queue_size=10)
	rate=rospy.Rate(1)
	msg_to_publish = test_custom_msg()
	counter=0
	while not rospy.is_shutdown():
		string_to_publish="Publishing %d"%counter
		counter+=1
		msg_to_publish.data=string_to_publish		
		msg_to_publish.counter=counter
		pub.publish(msg_to_publish)
		rospy.loginfo(msg_to_publish)
		rate.sleep()

if __name__=="__main__":
	rospy.init_node("simple_publisher")
	Publisher()