#!/usr/bin/env python3
import rospy
from epithet.msg import name

def Subscriber():
	sub=rospy.Subscriber('pub',name, callback_function)
	rospy.spin()

def callback_function(message):
	name=message.data
	rospy.loginfo("I heard "+name)


if __name__=="__main__":
	rospy.init_node("listener")
	Subscriber()