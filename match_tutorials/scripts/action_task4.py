#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from tf.transformations import quaternion_from_euler
import math

def move_base_client():
    #initialize node
    rospy.init_node('move_base_client')
    #create a simple action client using move base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    #wait for server first
    rospy.loginfo("Waiting for move base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move base server")
    return client

def create_move_base_goal(x,y,theta=0.0):
    #create a goal to send to the MoveBase server
    goal = MoveBaseGoal()
    
    #set target position
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    
    #set target orientation 
    quat = quaternion_from_euler(0.0, 0.0, theta)
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]
    
    return goal

def move_to_goal(client, x, y, theta=0.0):
    #create the goal
    goal = create_move_base_goal(x,y,theta)
    
    #define a feedback callback
    def feedback_cb(feedback):
        rospy.loginfo(f"Current position in feedback: {feedback.base_position.pose.position.x}, {feedback.base_position.pose.position.y}")
        
    #send the goal to the move_base server
    rospy.loginfo(f"Sending goal to coordinates x: {x}, y: {y}, theta: {theta}")
    client.send_goal(goal, feedback_cb=feedback_cb)
    
    #waits for the server to finish performing the action
    client.wait_for_result()
    
    #check if the goal was reached
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.logwarn("Failed to reach goal")


    
if __name__ == '__main__':
    try:
        client = move_base_client()
        
        #define the goal coordinates
        x_goal = 2.0
        y_goal = 2.0
        theta_goal = math.pi /2 
        
        #send the goal to move the robot
        move_to_goal(client, x_goal, y_goal, theta_goal)
        
        
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")