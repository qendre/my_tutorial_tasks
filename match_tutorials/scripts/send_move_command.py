#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

start_x = None
moving = True

def ground_truth_callback(msg):
    global start_x, moving
    
    if start_x is None:
        start_x = msg.pose.pose.position.x
        rospy.loginfo(f"Start position set to x: {start_x}")
    
    current_x = msg.pose.pose.position.x
    rospy.loginfo(f"Current x position: {current_x}")
    distance_moved = current_x - start_x
    rospy.loginfo(f"Distance moved: {distance_moved} meters")
    
    if distance_moved >= 3.0:
        stop_robot()
        moving = False
        
        
def move_forward():
    #Initialize publisher to send move commands
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('move_robot_1m', anonymous=True)

    #Subscribe to the /ground_truth topic to get position of the robot
    sub = rospy.Subscriber('/ground_truth', Odometry, ground_truth_callback)
    
    move_cmd = Twist()
    move_cmd.linear.x = 0.5 #velocity 
    
    #Publish the command
    rate = rospy.Rate(10)
    global moving
    
    while not rospy.is_shutdown() and moving:
        pub.publish(move_cmd)
        rospy.loginfo("Publishing move command: Move forward with velocity 0.5 m/s")
        rate.sleep()
    
    rospy.spin()
    
def stop_robot():
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    stop_cmd = Twist()
    pub.publish(stop_cmd)
    

if __name__ == '__main__':
    try:
        rospy.sleep(1)
        rospy.loginfo("Initializing robot movement...")
        move_forward()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")