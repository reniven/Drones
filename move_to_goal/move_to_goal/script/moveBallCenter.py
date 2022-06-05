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
from six.moves import xrange

class DroneCenterBall():
    def __init__(self):
        self

    def setUp(self):
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.pos = PoseStamped()
        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)

        self.state = State()
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)

    def send_pos(self):
        rate = rospy.Rate(10)
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

        
        #self.assertTrue(mode_set, (
           # "failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
           # format(mode, old_mode, timeout)))

    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

       # self.assertTrue(arm_set, ("failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2}".format(arm, old_arm, timeout)))

    def follow(self):
        """Test offboard position control"""
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        self.pos.pose.position.x = 0
        self.pos.pose.position.y = 0
        self.pos.pose.position.z = 0
        
        self.send_pos()
        self.pos.pose.position.x = 0
        self.pos.pose.position.y = 0
        self.pos.pose.position.z = 20

        self.send_pos()
        #self.thread()
        self.set_mode("AUTO.LAND", 5)
        self.set_arm(False, 5)
    
    def thread(self):
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

if __name__ == '__main__':
    rospy.init_node("center_node", anonymous=True)
    drone = DroneCenterBall()
    drone.setUp()
    drone.follow()

