#!/usr/bin/env python3
import rospy
from epithet.msg import name

def Publisher():
	pub=rospy.Publisher('pub',name, queue_size=10)
	rate=rospy.Rate(1)
	msg_to_publish = name()
	counter=0
	while not rospy.is_shutdown():
		string_to_publish="Leander"
		msg_to_publish.data=string_to_publish
		pub.publish(msg_to_publish)
		rospy.loginfo(msg_to_publish)
		rate.sleep()

if __name__=="__main__":
	rospy.init_node("talker")
	Publisher()