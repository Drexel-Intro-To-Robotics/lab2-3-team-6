#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import tf
import random as rand

class myTurtle():
    
    
    def __init__(self) -> None:
        """_summary_
        create all the nessary pubs/subs here and all the nessary other things
        """
        
        self.odom = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.goal = rospy.Subscriber('/goal', PoseStamped, self.nav_to_pose)
        self.send = rospy.Publisher('/goal', PoseStamped, queue_size=10)
        self.Twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        self.posx = 0
        self.posy = 0
        self.orient = 0
        
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.stop)
 
    
        

    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        rospy.sleep(1)

        #Saves Goal Data
        goalx = goal.pose.position.x
        goaly = goal.pose.position.y
        raworient = goal.pose.orientation
        goalorient = self.convert_to_euler(raworient)
        
        #Calculates which direction and normalizes for (-pi, pi)
        direction = math.atan2((goaly-self.posy), (goalx-self.posx))
        rospy.loginfo(f"Towards: {direction}")
        vel_msg = Twist()

        while not rospy.is_shutdown():
            error = direction - self.orient
            error = math.atan2((math.sin(error)),(math.cos(error)))

            if abs(error) < 0.03: #Break when close enough to the angle
                break
            
            #angular velocity optimization so that it gets slower the closer it gets to the right angle
            vel_msg.angular.z = 0.5 * error
            self.Twist.publish(vel_msg)
            self.rate.sleep()

        self.stop()
        rospy.loginfo("Pointing Towards Goal")

        #Calculate distance and send to drive_straight function
        distance = math.sqrt((self.posx - goalx)**2 +(self.posy - goaly)**2)
        self.drive_straight(distance, 0.2)
        rospy.loginfo("At Goal")

        while not rospy.is_shutdown():
            error = goalorient - self.orient
            error = math.atan2((math.sin(error)),(math.cos(error)))

            if abs(error) < 0.03: #Break when close enough to the angle
                break

            #angular velocity optimization like before
            vel_msg.angular.z = 0.5 * error 
            self.Twist.publish(vel_msg)
            self.rate.sleep()
            
        self.stop()
        rospy.loginfo("nav_to_pose complete")
        

    def odom_cb(self,msg:Odometry) ->None:
        """_summary_

        Get the odom and update the internal location of the robot
        Args:
            msg (Odometry): _description_
        """
        self.posx = msg.pose.pose.position.x
        self.posy = msg.pose.pose.position.y
        raworient = msg.pose.pose.orientation
        self.orient = self.convert_to_euler(raworient)
    
    
    def stop(self)->None:
        """_summary_
        
        Stop moving
        """
        rospy.loginfo("Stopping")
        vel_msg = Twist()
        
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.angular.z=0
        

        self.Twist.publish(vel_msg)
        rospy.loginfo("Stopped")
        
        
        
    def drive_straight(self, dist: float, vel: float)->None:
        """_summary_

        Args:
            dist (_type_): _description_
        """
        rospy.sleep(1)

        #Save current positions
        currentx = self.posx
        currenty = self.posy
        distance = 0

        vel_msg = Twist()
        vel_msg.linear.x=vel
        vel_msg.linear.y=0
        
        #While the robot has not traveled the given distance, keep moving
        rospy.loginfo(f"Forward: {dist}")
        while distance < dist and not rospy.is_shutdown():
            self.Twist.publish(vel_msg)
            self.rate.sleep()
            distance = math.sqrt((self.posx - currentx)**2 +(self.posy - currenty)**2)  #Calculate how far robot has gone

        self.stop()
        rospy.loginfo("Forward Done")

        
    
    def spin_wheels(self, u1, u2, time):
        """
        Spin the two wheels

        :param u1: wheel 1 speed (left)
        :param u2: wheel 2 speed (right)
        :param time: time to drive
        :return: None
        """
        rospy.sleep(1)
        T = 0.287 #Distance between wheels

        #Calculate linear and angular velocity
        linv = (u1 + u2)/2
        angv = (u2 - u1) / T


        vel_msg = Twist()
        vel_msg.linear.x = linv
        vel_msg.angular.z = angv

        #While time is not complete, keep sending velocities
        rospy.loginfo(f"u1 (left): {u1}, u2 (right): {u2}, time: {time}")
        start = rospy.get_time()
        while rospy.get_time() - start < time and not rospy.is_shutdown():
            self.Twist.publish(vel_msg)
            self.rate.sleep()

        self.stop()
        rospy.loginfo("Spin Wheel Done")
        

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return: None
        """
        rospy.sleep(1)

        lastO = self.orient
        rotation = 0

        #Changes which way to turn based on angle
        vel_msg = Twist()
        if angle > 0:
            vel_msg.angular.z = 0.1
        elif angle < 0:
            vel_msg.angular.z = -0.1
        else:
            vel_msg.angular.z = 0

        #While the rotation is not complete, keep rotating
        rospy.loginfo(f"Rotating: {angle}")
        while abs(rotation) < abs(angle) and not rospy.is_shutdown():
            self.Twist.publish(vel_msg)
            self.rate.sleep()


            currentO = self.orient
            delta = currentO - lastO
            delta = math.atan2(math.sin(delta), math.cos(delta)) #Normalizes the change so its between (-pi, pi)
            rotation = rotation + abs(delta)
            lastO = currentO
        
        self.stop()
        rospy.loginfo("Rotating Done")
        

    def drive_circle(self, radius):
        '''
        Drive in a circle
        Radius: Radius of circle in meters
        '''
        rospy.sleep(1)

        #Calculates angular velocity and time based on radius and 0.2 m/s
        w = 0.2 / radius
        length = 2 * math.pi * radius
        time = length / 0.2

        vel_msg = Twist()
        vel_msg.linear.x = 0.2
        vel_msg.angular.z = w

        start = rospy.get_time()
        rospy.loginfo(f"Circle Radius: {radius}")

        #While the time is not equal to the calculated time, keep sending velocity messages.
        while rospy.get_time() - start < time and not rospy.is_shutdown():
            self.Twist.publish(vel_msg)
            self.rate.sleep()

        self.stop()
        rospy.loginfo("Circle Done")
    
    def convert_to_euler(self, quat):
        # type: (Quaternion) -> float
        """
        This might be helpful to have
        :param quat: quaternion 
        :return: euler angles
        """
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw



def main():
    """_summary_
    create all the node start up here
    """
    rospy.init_node("turtlebot", anonymous = False)
    Turtle = myTurtle()
    rospy.sleep(1)
    #Task 5: Go in a Circle
    #rospy.loginfo("Task 5")
    #Turtle.drive_circle(0.5)

    #Task 6: Go in a Square
    #rospy.loginfo("Task 6")
    #for i in range (0, 4):
    #    Turtle.drive_straight(0.5, 0.1)
    #    Turtle.rotate(math.pi/2)

    #Task 8: Random Dance
    #rospy.loginfo("Task 8")
    #for i in range (0,4):
    #    Turtle.spin_wheels(rand.uniform(-0.2, 0.2), rand.uniform(-0.2, 0.2), rand.uniform(5, 15))
    #    rospy.sleep(1)
    
    rospy.spin()




if __name__ == '__main__':
    main()