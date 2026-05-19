#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower')

        # Publisher for robot motion commands
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers for the path and the robot's live position
        rospy.Subscriber('/astar/path', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Live tracking variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.waypoints = []
        
        # Controller tuning parameters
        self.linear_speed = 0.15   # Meters per second
        self.angular_speed = 0.4  # Radians per second
        self.distance_tolerance = 0.08 # How close to a waypoint counts as "arrived"
        self.angle_tolerance = 0.1    # How aligned the heading needs to be

        rospy.loginfo("Path Follower Node Initialized. Ready for path...")

    def odom_callback(self, msg):
        """Constantly updates the robot's live position and heading from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion orientation to simple 2D yaw angle (radians)
        orientation_q = msg.pose.pose.orientation
        quat_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = euler_from_quaternion(quat_list)

    def path_callback(self, msg):
        """Receives the calculated path from the A* planner"""
        rospy.loginfo("New path received! Unpacking waypoints...")
        self.waypoints = []
        
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.waypoints.append((x, y))
            
        self.execute_path()

    def execute_path(self):
        """Loops through all waypoints and drives to them sequentially"""
        rate = rospy.Rate(10) # 10 Hz control loop
        
        for index, waypoint in enumerate(self.waypoints):
            target_x, target_y = waypoint
            rospy.loginfo(f"Driving to waypoint {index + 1}/{len(self.waypoints)}: ({target_x:.2f}, {target_y:.2f})")

            while not rospy.is_shutdown():
                # 1. Calculate distance and heading to target
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                distance = math.sqrt(dx**2 + dy**2)
                
                desired_yaw = math.atan2(dy, dx)
                angle_error = desired_yaw - self.current_yaw

                # Normalize angle error to keep it between -pi and +pi
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

                # Check if we have successfully arrived at this waypoint
                if distance < self.distance_tolerance:
                    rospy.loginfo("Waypoint reached!")
                    break

                move_cmd = Twist()

                # 2. Control Logic: Prioritize turning to face the target first
                if abs(angle_error) > self.angle_tolerance:
                    # Turn in place toward the waypoint
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
                else:
                    # Drive straight forward toward the waypoint
                    move_cmd.linear.x = self.linear_speed
                    move_cmd.angular.z = 0.0

                self.cmd_pub.publish(move_cmd)
                rate.sleep()

        # Stop the robot entirely once the final goal is achieved
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        rospy.loginfo("Final Goal Reached! Navigation complete.")

if __name__ == '__main__':
    try:
        PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass