#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        self.last_distance = 0.0
        self.goal_distance = 0.0
        self.start_distance = 0.0
        self.dist_goal_active = False
        self.forward_movement = True

        self.goal_angle = 0.0
        self.start_angle = None
        self.angle_goal_active = False

        self.pose = Pose()

        # Position goal
        self.goal_position = None
        self.position_goal_active = False
        self.reached_orientation = False

        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)
        rospy.Subscriber("/goal_position", Point, self.position_callback)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        rospy.Timer(rospy.Duration(0.01), self.timer_callback)
        rospy.loginfo("Initialized straights_and_turns_turtle node!")
        rospy.spin()

    def pose_callback(self, msg):
        self.pose = msg

    def distance_callback(self, msg):
        self.last_distance = msg.data

    def goal_distance_callback(self, msg):
        self.goal_distance = msg.data
        if self.goal_distance != 0:
            self.start_distance = self.last_distance
            self.forward_movement = self.goal_distance > 0
            self.dist_goal_active = True
        rospy.loginfo("Received distance goal: %.2f", self.goal_distance)

    def goal_angle_callback(self, msg):
        self.goal_angle = msg.data
        if self.goal_angle != 0:
            self.start_angle = self.pose.theta
            self.angle_goal_active = True
        rospy.loginfo("Received angle goal: %.2f", self.goal_angle)

    def position_callback(self, msg):
        self.goal_position = msg
        self.position_goal_active = True
        self.reached_orientation = False
        rospy.loginfo("Received position goal: (%.2f, %.2f)", msg.x, msg.y)

    def angle_difference(self, current, target):
        diff = target - current
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def timer_callback(self, event):
        twist = Twist()
  ///........................./.''
        if self.dist_goal_active:
            moved = abs(self.last_distance - self.start_distance)
            if moved < abs(self.goal_distance):
                twist.linear.x = 1.0 if self.forward_movement else -1.0
            else:
                twist.linear.x = 0.0
                self.dist_goal_active = False
                rospy.loginfo("Distance goal reached.")

        elif self.angle_goal_active:
            turned = self.angle_difference(self.pose.theta, self.start_angle + self.goal_angle)
            if abs(turned) > 0.05:
                twist.angular.z = 1.0 if turned > 0 else -1.0
            else:
                twist.angular.z = 0.0
                self.angle_goal_active = False
                rospy.loginfo("Angle goal reached.")

        elif self.position_goal_active and self.goal_position is not None:
            dx = self.goal_position.x - self.pose.x
            dy = self.goal_position.y - self.pose.y
            distance_to_goal = math.sqrt(dx**2 + dy**2)
            angle_to_goal = math.atan2(dy, dx)
            angle_diff = self.angle_difference(self.pose.theta, angle_to_goal)

            if not self.reached_orientation:
                if abs(angle_diff) > 0.05:
                    twist.angular.z = 1.0 if angle_diff > 0 else -1.0
                else:
                    self.reached_orientation = True
            else:
                if distance_to_goal > 0.1:
                    twist.linear.x = 1.0
                else:
                    self.position_goal_active = False
                    rospy.loginfo("Position goal reached.")

        self.velocity_publisher.publish(twist)

if __name__ == '__main__':
    try:
        TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass