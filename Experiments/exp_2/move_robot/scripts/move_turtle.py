#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys

#/turtle1/Pose topic callback
def pose_callback(pose):
    rospy.loginfo("Robot X = %f : Y=%f :Z=%f\n",pose.x,pose.y,pose.theta)

#Function to move turtle: Linear and angular velocities are arguments
def move_turtle(lin_vel,ang_vel):
    rospy.init_node('move_turtle', anonymous=False)

    #The /turtle1/cmd_vel is the topic in which we have to send Twist messages

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    #Creating Twist message instance
    vel = Twist()

    while not rospy.is_shutdown():
        # Adding linear and angular velocity to the message
        
        vel.linear.x = lin_vel
        vel.angular.z = ang_vel
        
        rospy.loginfo("Linear Vel = %f: Angular Vel =%f",lin_vel,ang_vel)
        
        # Publishing Twist message  
        pub.publish(vel)
        
        rate.sleep()

if __name__ == "__main__":
    try:
        #Providing linear and angular velocity through command line    
        move_turtle(float(sys.argv[1]),float(sys.argv[2]))
    except rospy.ROSInterruptException:
        pass