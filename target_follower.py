#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, AprilTagDetectionArray

class TargetFollower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        # Change 'tainn' to your robot's name if needed
        self.cmd_vel_pub = rospy.Publisher('/prtc/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/prtc/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        self.last_tag_time = rospy.Time.now()
        rospy.spin()

    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = 0.0
        msg.omega = 0.0
        self.cmd_vel_pub.publish(msg)

    def move_robot(self, detections):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0

        now = rospy.Time.now()
        time_since_last_tag = (now - self.last_tag_time).to_sec()

        if len(detections) > 0:
            self.last_tag_time = now
            x = detections[0].transform.translation.x
            rospy.loginfo("AprilTag detected — x offset: %.5f", x)

            # Tuned parameters for smoother behavior
            Kp = 1.5                  # Lower speed
            max_omega = 2.0           # Limit max speed
            min_omega = 0.2           # Prevent friction stall
            dead_zone = 0.03
            hysteresis_zone = 0.06

            if abs(x) < dead_zone:
                rospy.loginfo("Tag centered — not rotating.")
                cmd_msg.omega = 0.0

            elif abs(x) > hysteresis_zone:
                omega = -Kp * x  # Invert to correct direction
                if abs(omega) < min_omega:
                    omega = min_omega * (1 if omega > 0 else -1)
                omega = max(min(omega, max_omega), -max_omega)
                rospy.loginfo("Tracking — rotating with omega: %.3f", omega)
                cmd_msg.omega = omega

            else:
                rospy.loginfo("In hysteresis zone — holding still.")
                cmd_msg.omega = 0.0

        else:
            # Seek mode only after no tag seen for a short duration
            if time_since_last_tag > 0.5:
                rospy.loginfo("No tags seen for %.2fs — slowly seeking...", time_since_last_tag)
                cmd_msg.omega = 0.8  # Slower seek
            else:
                rospy.loginfo("Waiting before seeking (%.2fs)...", time_since_last_tag)
                cmd_msg.omega = 0.0

        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        TargetFollower()
    except rospy.ROSInterruptException:
        pass
