#!/usr/bin/env python3

import sys
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        # Update with your robot name if needed
        self.image_sub = rospy.Subscriber(
            '/prtc/camera_node/image/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1
        )

        rospy.init_node("my_lane_detector")

    def image_callback(self, msg):
        rospy.loginfo("image_callback")

        # Convert to OpenCV image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Crop the image
        height, width = img.shape[:2]
        cropped = img[int(height / 2):, :]

        # Apply Gaussian blur to reduce noise before processing
        blurred = cv2.GaussianBlur(cropped, (5, 5), 0)

        # Convert to HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # --- Improved White Filter ---
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # --- Improved Yellow Filter ---
        lower_yellow = np.array([18, 130, 130])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Morphological Opening (larger kernel)
        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

        # Create filtered outputs
        white_result = cv2.bitwise_and(cropped, cropped, mask=white_mask)
        yellow_result = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

        # Display the results
        cv2.imshow('Cropped Image', cropped)
        cv2.imshow('White Lane Filter', white_result)
        cv2.imshow('Yellow Lane Filter', yellow_result)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
