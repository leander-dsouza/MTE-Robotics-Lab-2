#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

X = 0.0
Y = 0.0
yaw = 0.0


def pose_callback(pose):
    global X, Y, yaw
    rospy.loginfo("X=%f, Y=%f\n", pose.x, pose.y)
    X = pose.x
    Y = pose.y
    yaw = pose.theta


def move(speed, distance, is_forward):
    velocity_message = Twist()

    global X, Y
    X0 = X
    Y0 = Y

    if is_forward:
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    while True:
        rospy.loginfo("Turtlesim moves forward")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

        # rospy.loginfo("%f %f %f %f", X,Y,X0,Y0)

        distance_moved = math.sqrt(((X - X0) ** 2) + ((Y - Y0) ** 2))
        print(distance_moved,X,Y,X0,Y0)

        if not (distance_moved < distance):
            rospy.loginfo("reached")
            rospy.logwarn("Stopping the Robot")
            break

    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def rotate(angular_speed_degree, relative_angle_degree, clockwise):
    global yaw

    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0

    theta0 = yaw
    angular_speed = math.radians(abs(angular_speed_degree))

    if clockwise:
        velocity_message.angular.z = -abs(angular_speed)

    else:
        velocity_message.angular.z = abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(10)
    and_vel_topic = '/turtle1/cmd_vel'

    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    t0 = rospy.Time.now().to_sec()

    while (True):

        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1 - t0) * angular_speed_degree
        loop_rate.sleep()

        if current_angle_degree > relative_angle_degree:
            rospy.loginfo("reached")
            break

    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def go_to_goal(x_goal, y_goal):
    global X
    global Y, yaw

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while True:
        K_linear = 0.5

        distance = abs(math.sqrt(((x_goal - X) ** 2) + ((y_goal - Y) ** 2)))
        linear_speed = distance * K_linear
        K_angular = 4.0

        desired_angle_goal = math.atan2(y_goal - Y, x_goal - X)
        angular_speed = (desired_angle_goal - yaw) * K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)

        print('x=', X, 'y=', Y)

        if distance < 0.01:
            break


def setDesiredOrientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw

    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0

    print(relative_angle_radians)
    print(desired_angle_radians)
    rotate(30, math.degrees(abs(relative_angle_radians)), clockwise)


def hexagon(side_length):

    for _ in range(6):

        move(1.0,3.0,True)
        rotate(10,60,False)
    




if __name__ == '__main__':

    try:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        cmd_vel_topic = '/turtle1/cmd_vel'

        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        position_topic = '/turtle1/pose'

        rospy.Subscriber(position_topic, Pose, pose_callback)
        time.sleep(1)

        hexagon(3.0)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")