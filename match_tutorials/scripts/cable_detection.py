#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Initialize the CvBridge
bridge = CvBridge()

def image_callback(color_msg):
    try:
        # Step 1: Convert ROS message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(color_msg, "bgr8")

        # Step 2: Convert the image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Step 3: Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Step 4: Perform Canny edge detection
        edges = cv2.Canny(blurred, 100, 200)

        # Step 5: Use morphological operations to close gaps in edges
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # Step 6: Find Contours and Filter
        contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Filter small contours
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                if 0.2 < aspect_ratio < 5:  # Long and thin shape filtering
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(cv_image, "Cable", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Step 7: Visualize Results
        cv2.imshow("Original Image with Cable Detection", cv_image)
        cv2.imshow("Edges (Closed)", closed_edges)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(f"Error processing image: {str(e)}")

def main():
    rospy.init_node("cable_detector_rgb_only")

    # Subscribe to the RGB topic from the RealSense camera
    color_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

    rospy.loginfo("Cable Detector Node Started (RGB Only)")
    rospy.spin()

if __name__ == "__main__":
    main()
