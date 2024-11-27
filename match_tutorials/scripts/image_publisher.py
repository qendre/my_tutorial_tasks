#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Display the image
        cv2.imshow("Image Raw", cv_image)
        
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Display the grayscale image
        cv2.imshow("Grayscale Image", gray_image)
        
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        cv2.imshow("HSV Image", hsv_image)
        
        # Apply Canny Edge Detection
        edges = cv2.Canny(gray_image, 100, 200)  # Threshold values can be adjusted
        
        # Display the edge-detected image
        cv2.imshow("Edge Detection", edges)
        
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(f"Error converting image: {str(e)}")

if __name__ == "__main__":
    rospy.init_node("image_listener")
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()
