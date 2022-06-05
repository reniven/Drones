#!/usr/bin/env python
import rospy
from ball_detector.msg import ballLocation
from std_msgs.msg import String

def listener():
    rospy.init_node('listener', anonymous=True)
    #while not rospy.is_shutdown():
    rospy.Subscriber("/ballLocation", ballLocation, callback)
    #things = ballLocation()
    #while not rospy.is_shutdown():
    #    print(things)
    #    print(things.x)
    #    print(things.y)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
def callback(data):
    #rospy.loginfo("x: " + str(data.x) + " y: " + str(data.y))
    print(data)

if __name__ == '__main__':
    listener()
