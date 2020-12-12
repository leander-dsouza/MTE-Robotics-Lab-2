#!/usr/bin/env python3

import  rospy
from test_pub_sub.msg import test_custom_msg

def Subscriber():
	sub=rospy.Subscriber('string_publish', test_custom_msg, callback_function)
	rospy.spin()

def callback_function(message):
	string_rec=message.data
	counter_rec=message.counter
	rospy.loginfo("I received: %d"%counter_rec)

if __name__=="__main__":
	rospy.init_node("simple_subscriber")
	Subscriber()