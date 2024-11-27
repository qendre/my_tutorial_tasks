#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def detect_and_draw_cables(image, hsv_image):
    # Define color ranges in HSV
    color_ranges = {
        "red": [(0, 120, 70), (10, 255, 255), (170, 120, 70), (180, 255, 255)],
        "yellow": [(20, 100, 100), (30, 255, 255)],
        "blue": [(100, 150, 50), (140, 255, 255)]
    }

    # Colors for bounding boxes
    box_colors = {
        "red": (0, 0, 255),
        "yellow": (0, 255, 255),
        "blue": (255, 0, 0)
    }

    for color, ranges in color_ranges.items():
        if color == "red":
            # Handle red's two ranges
            mask1 = cv2.inRange(hsv_image, ranges[0], ranges[1])
            mask2 = cv2.inRange(hsv_image, ranges[2], ranges[3])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            # Other colors (single range)
            mask = cv2.inRange(hsv_image, ranges[0], ranges[1])

        # Morphological operations to clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 50:  # Filter by size
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                # Draw bounding box on the original image
                cv2.rectangle(image, (x, y), (x + w, y + h), box_colors[color], 2)
                # Optionally add a label
                cv2.putText(image, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_colors[color], 2)

    return image

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Convert to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Detect and draw cables
        result_image = detect_and_draw_cables(cv_image, hsv_image)
        
        # Display the result
        cv2.imshow("Detected Cables", result_image)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(f"Error processing image: {str(e)}")

if __name__ == "__main__":
    rospy.init_node("color_detector")
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()