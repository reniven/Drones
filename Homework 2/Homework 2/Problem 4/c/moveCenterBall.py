#!/usr/bin/env python
#Taken from the mavros example 
import rospy
from ball_detector.msg import ballLocation
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
                            WaypointList
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, \
                            WaypointPush
from threading import Thread
from std_msgs.msg import Header
from pymavlink import mavutil
from six.moves import xrange
import unittest

class DroneCenterBall():
    def __init__(self):
        self.pos = PoseStamped()
        self.radius = 1
        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.state = State()
        self.extended_state = ExtendedState()

        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def set_arm(self, arm, timeout):
        rospy.loginfo("Setting FCU Arm: {0}".format(arm))
        if self.state.armed == arm:
            rospy.loginfo("Set Arm Success")
        else:
            try:
                res = self.set_arming_srv(arm)
                if not res.success:
                    rospy.logerr("Failed to send arm command")
            except rospy.ServiceException as e:
                rospy.loginfo(e)
    
    def set_mode(self, mode, timeout):
        rospy.loginfo("Setting FCU mode: {0}".format(mode))

        for i in xrange(timeout):
            if self.state.mode == mode:
                rospy.loginfo("Set Mode Success")
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)
                    if not res.mode_sent:
                        rospy.logerr("Failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

    def follow(self):
        rospy.init_node('center_ball')

        self.set_mode("OFFBOARD", 5)
        self.set_arm("armed", 5)

        
        while not rospy.is_shutdown():
            rospy

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    drone = DroneCenterBall()
    drone.test_follow()
