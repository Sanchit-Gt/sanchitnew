#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

LINEAR_SPEED = 2.0
ANGULAR_SPEED = math.pi 
TRAVEL_DISTANCE = 2.0

def move_turtle_square():
    rospy.init_node('turtlesim_square_node', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    def move(linear=0.0, angular=0.0, duration=0.0):
        start = rospy.Time.now().to_sec()
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        while rospy.Time.now().to_sec() - start < duration and not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        pub.publish(msg)
        rate.sleep()

    while not rospy.is_shutdown():
        move(linear=LINEAR_SPEED, duration=TRAVEL_DISTANCE / LINEAR_SPEED)
        move(angular=ANGULAR_SPEED, duration= (math.pi +0.5)/ ANGULAR_SPEED)
        move(linear=LINEAR_SPEED, duration=TRAVEL_DISTANCE / LINEAR_SPEED)
        move(angular=ANGULAR_SPEED, duration= (math.pi +0.5) / ANGULAR_SPEED)
        move(linear=LINEAR_SPEED, duration=TRAVEL_DISTANCE / LINEAR_SPEED)
        move(angular=ANGULAR_SPEED, duration= (math.pi+0.5) / ANGULAR_SPEED)
        move(linear=LINEAR_SPEED, duration=TRAVEL_DISTANCE / LINEAR_SPEED)
        move(angular=ANGULAR_SPEED, duration= (math.pi +0.5) / ANGULAR_SPEED)


if __name__ == '__main__':
    try:
        move_turtle_square()
    except rospy.ROSInterruptException:
        pass