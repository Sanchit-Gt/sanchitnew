#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, AprilTagDetectionArray

class TargetFollower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=False)
        rospy.on_shutdown(self.clean_shutdown)

        # Get robot name from launch namespace
        self.robot_name = rospy.get_namespace().strip('/')

        # Publisher for movement commands
        self.cmd_pub = rospy.Publisher(
            f'/{self.robot_name}/car_cmd_switch_node/cmd',
            Twist2DStamped, queue_size=1
        )

        # Subscriber for AprilTag detections
        rospy.Subscriber(
            f'/{self.robot_name}/apriltag_detector_node/detections',
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=1
        )

        # Internal variables
        self.no_detection_counter = 0
        self.detection_threshold = 5  # number of empty frames before seeking
        self.cmd = Twist2DStamped()
        self.rate = rospy.Rate(10)  # 10 Hz

        rospy.loginfo(" TargetFollower started for robot: /%s", self.robot_name)
        rospy.spin()

    def tag_callback(self, msg):
        # No detections? Keep seeking
        if len(msg.detections) == 0:
            self.no_detection_counter += 1
            rospy.loginfo("No tag seen. Seeking... (%d)", self.no_detection_counter)
            if self.no_detection_counter >= self.detection_threshold:
                self.seek_object()
            return

        # Tag detected → reset seeking counter
        self.no_detection_counter = 0

        # Use the first tag detected
        tag = msg.detections[0]
        x_offset = tag.transform.translation.x

        rospy.loginfo(f"Tag detected - x offset: {x_offset:.3f}")
        self.look_at_object(x_offset)

    def seek_object(self):
        # Rotate in place to find a tag
        self.cmd.v = 0.0
        self.cmd.omega = 1.0  # constant spin
        self.cmd_pub.publish(self.cmd)
        self.rate.sleep()

    def look_at_object(self, x_offset):
        gain = 1.0
        min_omega = 0.5
        max_omega = 2.0

        # Dead zone to stop jitter if tag is nearly centered
        if abs(x_offset) < 0.05:
            omega = 0.0
            rospy.loginfo("Tag centered — stopping rotation.")
        else:
            omega = -gain * x_offset

            # Clamp values
            if abs(omega) < min_omega:
                omega = min_omega if omega > 0 else -min_omega
            omega = max(-max_omega, min(max_omega, omega))

        self.cmd.v = 0.0
        self.cmd.omega = omega
        self.cmd_pub.publish(self.cmd)
        self.rate.sleep()

    def clean_shutdown(self):
        rospy.loginfo("Shutdown: stopping robot.")
        self.cmd.v = 0.0
        self.cmd.omega = 0.0
        self.cmd_pub.publish(self.cmd)

if __name__ == '__main__':
    try:
        TargetFollower()
    except rospy.ROSInterruptException:
        pass