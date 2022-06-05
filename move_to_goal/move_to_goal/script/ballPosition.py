#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
def ballPosition():
    rospy.init_node("ball_position", anonymous=True)
    while not rospy.is_shutdown():
        rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    
def callback(data):
    pub = rospy.Publisher("/ballPosition", Pose)
    while not rospy.is_shutdown():
        pub.publish(data.pose[1])

if __name__ == '__main__':
    ballPosition()
