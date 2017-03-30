#!/usr/bin/env python
import rospy

from Elmo import *
from joint import *
from follow_controller import *
from publishers import *

if __name__ == "__main__":
    rospy.init_node('larm_controller')
    robotjoints = rospy.get_param('~robotjoints')
    joints_instance = list()
    for jointname in robotjoints:
        joints_instance.append(Joint(jointname))

    joints_dict = dict(zip(robotjoints, joints_instance))
    elmo_controller = Elmo('elmo_driver', joints_instance)
    a = FollowController('left_arm_controller', elmo_controller, joints_dict)

    joint_state_publisher = JointStatePublisher()

    rospy.loginfo("Started FollowController ")

    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
#        hello_str = "hello world %s" % rospy.get_time()
#        rospy.logwarn(hello_str)
        elmo_controller.UpdateRead()
        joint_state_publisher.update(joints_instance)

        rate.sleep()


