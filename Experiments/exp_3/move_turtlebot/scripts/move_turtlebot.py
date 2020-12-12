#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys


X=0

def pose_callback(msg):
	global  X
	X=msg.pose.pose.position.x
	rospy.loginfo("Robot X=%f\n",X)

def move(lin_vel,ang_vel,distance):
	global X
	rospy.init_node('move_turtlebot',anonymous=False)
	pub=rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=10)
	rospy.Subscriber('/odom',Odometry,pose_callback)
	rate=rospy.Rate(10)
	vel=Twist()
	while not rospy.is_shutdown():
		vel.linear.x=lin_vel
		vel.linear.y=0
		vel.linear.z=0

		vel.angular.x=0
		vel.angular.y=0
		vel.angular.z=ang_vel

		#rospy.loginfo("Linear Vel=%f: Angular Vel=%f",lin_vel,ang_vel)

		if(X>=distance):
			rospy.loginfo("Robot Reached Destination")
			rospy.logwarn("Stopping robot")
			break
		pub.publish(vel)
		rate.sleep()

if __name__=='__main__':
	try:
		move(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]))

	except rospy.ROSInterruptException:
		pass	
