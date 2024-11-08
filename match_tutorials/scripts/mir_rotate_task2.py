#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

class RobotController:
    
    #States 
    SET_END_POSITION = 0
    MOVE_TO_END_POSITION = 1
    ROTATE_TO_NEW_ORIENTATION = 2
    MOVE_TO_START_POSITION = 3

    def __init__(self):
        rospy.init_node('mir_rotate')
        self.pub=rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_callback)
        
        #Initialize state and pose variables
        self.current_state = self.SET_END_POSITION
        self.current_pose = None
        self.start_position = None
        self.end_position = None
        self.target_orientation = None
        self.distance_threshold = 0.1
        self.angle_threshold = 0.05
        
    def odom_callback(self,msg):
        if self.current_pose is None:
            self.start_position = msg.pose.pose  # Save the initial position as the start position
            rospy.loginfo(f"Start Position set to: {self.start_position.position.x}, {self.start_position.position.y}")
        self.current_pose = msg.pose.pose
        #rospy.loginfo(f"Current Position: {self.current_pose.position.x}, {self.current_pose.position.y}")

        
    def calculate_distance(self, pose1, pose2):
        return math.sqrt((pose2.position.x - pose1.position.x)**2 + (pose2.position.y - pose1.position.y)**2)
        

        
    def get_yaw_from_pose(self,pose):
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list) #ignore roll and pitch, only yaw is relevant to rotate 180 degree
        return yaw
        
    def rotate_robot(self, target_yaw):
        twist = Twist()
        current_yaw = self.get_yaw_from_pose(self.current_pose)
        angle_diff = target_yaw - current_yaw
        
        # Normalize the angle difference
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Rotate until within the angle threshold
        if abs(angle_diff) > self.angle_threshold:
            twist.angular.z = 0.5 * (angle_diff / abs(angle_diff))  # Rotate in the direction of the angle difference
        else:
            twist.angular.z = 0  # Stop rotation when aligned

        self.pub.publish(twist)
        
    def move_forward(self,target_position):
        twist = Twist()
        distance = self.calculate_distance(self.current_pose, target_position)
        
        rospy.loginfo(f"Distance to target: {distance}")
        
        if distance > self.distance_threshold:
            twist.linear.x = 0.2
        else:
            twist.linear.x = 0
            
        self.pub.publish(twist)
        
    #State functions
    def set_end_position(self):
        self.start_position = self.current_pose
        self.end_position = Pose()
        self.end_position.position.x = self.start_position.position.x + 2
        self.end_position.position.y = self.start_position.position.y
        self.current_state = self.MOVE_TO_END_POSITION
        
    def move_to_end_position(self):
        self.move_forward(self.end_position)
        if self.calculate_distance(self.current_pose, self.end_position) < self.distance_threshold:
            self.current_state = self.ROTATE_TO_NEW_ORIENTATION
            self.target_orientation = self.get_yaw_from_pose(self.current_pose) + math.pi
            
    def rotate_to_new_orientation(self):
        self.rotate_robot(self.target_orientation)
        if abs(self.get_yaw_from_pose(self.current_pose) - self.target_orientation) <self.angle_threshold:
            self.current_state = self.MOVE_TO_START_POSITION
            
    def move_to_start_position(self):
        self.move_forward(self.start_position)
    
        distance_to_start = self.calculate_distance(self.current_pose, self.start_position)
        rospy.loginfo(f"Distance to start position: {distance_to_start}")
    
        if distance_to_start < self.distance_threshold:
            rospy.loginfo("Returned to starting position")
            rospy.signal_shutdown("Task complete")
            
    def run(self):
        rate = rospy.Rate(10)
        
        state_functions = {
            self.SET_END_POSITION: self.set_end_position,
            self.MOVE_TO_END_POSITION: self.move_to_end_position,
            self.ROTATE_TO_NEW_ORIENTATION: self.rotate_to_new_orientation,
            self.MOVE_TO_START_POSITION: self.move_to_start_position
        }
        
        while not rospy.is_shutdown():
            if self.current_pose is None:
                continue  # Wait until we have a valid pose

            # Call the function associated with the current state
            rospy.loginfo(f"Current state: {self.current_state}") 
            state_functions[self.current_state]()

            rate.sleep()

if __name__ == '__main__':
    try:
        robot_controller = RobotController()
        robot_controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")