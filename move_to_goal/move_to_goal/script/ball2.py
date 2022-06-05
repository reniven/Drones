#!/usr/bin/env python
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

PKG = 'px4'

class DroneCenterBall():
    def __init__(self):


        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")

        self.pos = PoseStamped()
        self.radius = 1
        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.state = State()
        rospy.Subscriber('mavros/state', State, self.callback)
        self.extended_state = ExtendedState()

    def set_arm(self, arm, timeout):
        rospy.loginfo("Setting FCU Arm: {0}".format(arm))
        for i in xrange(timeout):
            print(self.state.armed)
            if self.state.armed == arm:
                rospy.loginfo("Set Arm Success")
            else:
                try:
                    res = self.set_arming_srv(arm)
                    print(res.success)
                    if not res.success:
                        rospy.logerr("Failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.loginfo(e)
    
    def set_mode(self, mode, timeout):
        rospy.loginfo("Setting FCU mode: {0}".format(mode))

        for i in xrange(timeout):
            print(self.state_sub.mode)
            print(mode)
            if self.state_sub.mode == mode:
                rospy.loginfo("Set Mode Success")
                break
            else:
                print("banana")
                try:
                    print("Hello")
                    res = self.set_mode_srv(0, mode)
                    print("Idk")
                    print(res.mode_sent)
                    if not res.mode_sent:
                        print("hkdfd")
                        rospy.logerr("Failed to send mode command")
                        print("chicken")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
                    print(e)
            print(self.state_sub.system_status)

    def test_follow(self):
        print("hello")
        print(self.state) 
        self.state.armed = True
        print(self.state)
        rate = rospy.Rate(20.0)
        #while not rospy.is_shutdown():
            #rate.sleep()

        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

    def shutdown():
        print("Shutting down")

    def callback(data):
        self.state = data.armed
if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    #rostest.rosrun(PKG, 'test_follow',
     #              DroneCenterBall)
    drone = DroneCenterBall()
    drone.test_follow()

