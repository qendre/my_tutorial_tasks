#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import tf2_geometry_msgs
import tf2_py
from tf.transformations import euler_from_quaternion
import math

# Global variables for position and orientation
p_current = Pose()
AMCLx = Pose()
GTx = Pose()

q_current = Quaternion()
AMCLo = Quaternion()
GTo = Quaternion()

# Variables for publishing twist and end goal pose
twist = Twist()
p_end = Pose()
yaw_end = 0.0
yaw_current = 0.0
state = 0
round_num = 1

# Publisher
pub  = None

def move(pose):
    global state, twist, p_end
    # Move the robot towards the end goal
    if abs(p_end.position.x - pose.position.x) > 0.1:
        twist.linear.x = 0.3
    else:
        twist.linear.x = 0
        state += 1  # Move to the next state
    
def turn(quat):
    global state, twist, yaw_end, yaw_current
    orientation_list = [quat.x, quat.y, quat.z, quat.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    if abs(yaw_end - yaw_current) > 0.05:
        twist.angular.z = 0.5
    else:
        twist.angular.z = 0
        state +=1 #move to the next state

def run(pose,quat):
    global state, p_end, yaw_end, round_num, pub
    rospy.loginfo(f"Running state: {state}")
    
    if state == 0:
        p_end = pose
        p_end.position.x = 1.0
        state += 1
        
    elif state == 1:
        move (pose)
        
    elif state == 2:
        rospy.loginfo(f"End position x: {p_end.position.x}, Current x: {pose.position.x}")
        yaw_end = math.pi  # 180 degrees in radians
        turn(quat)
        
    elif state == 3:
        p_end = pose
        p_end.position.x = 0.0
        state +=1
        
    elif state == 4:
        move (pose)
        
    elif state == 5:
        yaw_end = 0.0
        turn(quat)
    else:
        state =0
        round_num += 1
    
            
def odom_callback(msg):
    global p_current, q_current, round_num
    
    p_current=msg.pose.pose
    q_current=msg.pose.pose.orientation
    if round_num % 3 == 1:
        rospy.loginfo("Using Odometry")
        run(p_current,q_current)

def amcl_callback(msgAMCL):
    global AMCLx, AMCLo, round_num
    
    AMCLx = msgAMCL.pose.pose
    AMCLo = msgAMCL.pose.pose.orientation
    if round_num % 3 == 2:
        rospy.loginfo("Using AMCL")
        run(AMCLx, AMCLo)
    
def gt_callback(msgGT):
    global GTx, GTo, round_num
    
    GTx=msgGT.pose.pose
    GTo=msgGT.pose.pose.orientation
    
    if round_num % 3 == 0:
        rospy.loginfo("Using Ground Truth")
        run(GTx,GTo)
    
    
def main():
    global pub
    rospy.init_node("solution_node3", anonymous=True)
    pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
    
    #subscribe to pose topics
    rospy.Subscriber("/mobile_base_controller/odom", Odometry, odom_callback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_callback)
    rospy.Subscriber("/ground_truth", Odometry, gt_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")