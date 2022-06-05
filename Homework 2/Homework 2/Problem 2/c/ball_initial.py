#!/usr/bin/env python
import rospy
#from forces_torques.srv import *
from gazebo_msgs.srv import ApplyBodyWrench
import geometry_msgs.msg
import time

def move(body_name1, wrench1):
    
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        abw = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        duration1 = rospy.Duration(secs = 1, nsecs = 0)
        abw(body_name = body_name1, wrench = wrench1, duration = duration1)
    except rospy.ServiceException:
        pass

def move_E_hori():
    body_name = "unit_sphere::link"
    wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 1, 
                y = 0, z = 0), 
                torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))
    move(body_name, wrench)
    
    time.sleep(3)
    wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = -1,
                y = 0, z = 0), 
                torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))

    move(body_name, wrench)
    
    time.sleep(1)
    wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = -1,
                y = 0, z = 0),
                torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))
    move(body_name, wrench)

    time.sleep(3)
    wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 1,
                y = 0, z = 0),
                torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))

    move(body_name, wrench)
    time.sleep(3)

def move_E_vert():
    body_name = "unit_sphere::link"
    wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0,
                y = 1, z = 0),
                torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))
    move(body_name, wrench)

    time.sleep(3)
    wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0,
                y = -1, z = 0),
                torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))

    move(body_name, wrench)
    time.sleep(3)

def move_L():
    body_name = "unit_sphere::link"
    wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0,
                y = -3, z = 0),
                torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))
    move(body_name, wrench)
    wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 2,
                y = 0, z = 0),
                torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))
    move(body_name, wrench)

if __name__ == "__main__":
    try:
        move_E_hori()
        move_E_vert()
        move_E_hori()
        move_E_vert()
        move_E_hori()
        print("Completed")
    except rospy.ROSInterruptException:
        pass