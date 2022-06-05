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

class DroneCenterBall():
    def __init__(self):

        self.pos = PoseStamped()
        self.radius = 1
        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.state = State()
        self.extended_state = ExtendedState()
    
    def setup(self):
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

    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def callback(self, data):
        self.data2 = data

    def follow(self):
        rospy.init_node('center_ball')

        self.set_mode("OFFBOARD", 5)
        self.set_arm("armed", 5)

        rospy.Subscriber("ball_detector/ballLocation", ballLocation, callback)

        while not rospy.is_shutdown():
            if(data.x != 0 and doneX != True):
                self.pos.pose.position.x = x
                self.pos.pose.position.y = y
                self.pos.pose.position.z = z

                if(data.x == 0):
                    doneX = True
            if(data.y != 0 and doneX == True):
                self.pos.pose.position.x = x
                self.pos.pose.position.y = y
                self.pos.pose.position.z = z

                
        rospy.spin()
            

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    #rostest.rosrun(PKG, 'test_follow',
     #              DroneCenterBall)
    drone = DroneCenterBall()
    drone.test_follow()
