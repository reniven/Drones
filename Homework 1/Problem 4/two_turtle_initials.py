#!/usr/bin/env python
#Based upon the codes given on the https://wiki.ros.org/turtlesim/Tutorials/ website
import rospy
from geometry_msgs.msg import Twist

def create_node():
    # Starts a new node
    rospy.init_node('turle_initials', anonymous=True)
    publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    publisher2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    coordinates = Twist()

    return publisher, coordinates, publisher2

#function for moving the turtle
def move(publisher, coordinates, x):

    #Since we are moving just in x-axis
    coordinates.linear.x = x
    coordinates.linear.y = 0
    coordinates.linear.z = 0
    coordinates.angular.x = 0
    coordinates.angular.y = 0
    coordinates.angular.z = 0

    #Checks to make sure the distance has been met
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while(current_distance < 2):

        publisher.publish(coordinates)
        t1=rospy.Time.now().to_sec()
        current_distance= x*(t1-t0)

    #Stops the robot
    coordinates.linear.x = 0
    publisher.publish(coordinates)

#function for rotating the turtle
def rotate(publisher, coordinates, z):

    #Set the values for linear and angular
    #Given that this is a rotate, linear will all be 0
    coordinates.linear.x=0
    coordinates.linear.y=0
    coordinates.linear.z=0
    coordinates.angular.x = 0
    coordinates.angular.y = 0
    coordinates.angular.z = z

    #Checks for the angle to make sure that its bet met
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < abs(z)):
        publisher.publish(coordinates)
        t1 = rospy.Time.now().to_sec()
        current_angle = abs(z)*(t1-t0)


    #Forcing our robot to stop at the space
    coordinates.angular.z = 0
    publisher.publish(coordinates)

if __name__ == '__main__':
    try:
        #Runs the routine for EL initials for turtle1
        publisher, coordinates, publisher2 = create_node()
        rotate(publisher, coordinates, 4)
        move(publisher, coordinates,1)
        rotate(publisher, coordinates, -1.5)
        move(publisher, coordinates,1)
        rotate(publisher, coordinates,-1.5)
        move(publisher, coordinates,1)
        rotate(publisher, coordinates,3)
        move(publisher, coordinates,1)
        rotate(publisher, coordinates,-1.5)
        move(publisher, coordinates,1)
        rotate(publisher, coordinates,-1.5)
        move(publisher, coordinates,2)
        rotate(publisher, coordinates,-1.5)
        move(publisher, coordinates,2)
        rotate(publisher, coordinates,1.5)
        move(publisher, coordinates,2)
        
        #Runs the routine for EL initials for turtle2
        rotate(publisher2, coordinates, 4)
        move(publisher2, coordinates,1)
        rotate(publisher2, coordinates, -1.5)
        move(publisher2, coordinates,1)
        rotate(publisher2, coordinates,-1.5)
        move(publisher2, coordinates,1)
        rotate(publisher2, coordinates,3)
        move(publisher2, coordinates,1)
        rotate(publisher2, coordinates,-1.5)
        move(publisher2, coordinates,1)
        rotate(publisher2, coordinates,-1.5)
        move(publisher2, coordinates,2)
        rotate(publisher2, coordinates,-1.5)
        move(publisher2, coordinates,2)
        rotate(publisher2, coordinates,1.5)
        move(publisher2, coordinates,2)

    except rospy.ROSInterruptException: 
        pass