#!/usr/bin/env python

import rospy, actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from diagnostic_msgs.msg import *


class FollowController:
    """ A controller for joint chains, exposing a FollowJointTrajectory action. """

    def __init__(self, name,device,joints_dict):
        self.interpolating = 0

        self.name = name
        self.device = device
        self.joints_dict = joints_dict

        # parameters: rates and joints
        self.rate = rospy.get_param('~'+name+'/rate',50.0)
        self.joints = rospy.get_param('~elmo_driver/joints')

        # action server
        actionname= rospy.get_param('~'+name+'/action_name','follow_joint_trajectory')
        self.server = actionlib.SimpleActionServer('/'+name+'/'+actionname , FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=False)

        rospy.loginfo("Started FollowController ("+name +"). Joints: " + str(self.joints) + " on C" )

        self.server.start()

    def startup(self):
        self.server.start()

    def actionCb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved.")
        traj = goal.trajectory

        if set(self.joints) != set(traj.joint_names):
            for j in self.joints:
                if j not in traj.joint_names:
                    msg = "Trajectory joint names does not match action controlled joints." + str(traj.joint_names)
                    rospy.logerr(msg)
                    self.server.set_aborted(text=msg)
                    return
            rospy.logwarn("Extra joints in trajectory")

        if not traj.points:
            msg = "Trajectory empy."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            msg = "Trajectory invalid."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        if self.executeTrajectory(traj):   
            self.server.set_succeeded()
        else:
            self.server.set_aborted(text="Execution failed.")

        rospy.loginfo(self.name + ": Done.")
    
    def commandCb(self, msg):
        # don't execute if executing an action
        if self.server.is_active():
            rospy.loginfo(self.name+": Received trajectory, but action is active")
            return
        self.executing = True
        self.executeTrajectory(msg)
        self.executing = False    

    def executeTrajectory(self, traj):
        rospy.loginfo("Executing trajectory")
        rospy.logdebug(traj)
        # carry out trajectory
        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            rospy.logerr("Invalid joint in trajectory.")
            return False

        # get starting timestamp, MoveIt uses 0, need to fill in
        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = rospy.Time.now()

        r = rospy.Rate(self.rate)
        last = [ self.joints_dict[joint].position for joint in self.joints ]
        for point in traj.points:
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)

            desired = [ point.positions[k] for k in indexes ]
            endtime = start + point.time_from_start
            while rospy.Time.now() + rospy.Duration(0.01) < endtime:
                err = [ (d-c) for d,c in zip(desired,last) ]
                velocity = [ abs(x / (self.rate * (endtime - rospy.Time.now()).to_sec())) for x in err ]
                rospy.logdebug(err)
                for i in range(len(self.joints)):
                    if err[i] > 0.001 or err[i] < -0.001:
                        cmd = err[i] 
                        top = velocity[i]
                        if cmd > top:
                            cmd = top
                        elif cmd < -top:
                            cmd = -top
                        last[i] += cmd

                        self.device.setJoint(self.joints[i],last[i])

                        #rospy.logwarn("%s:%f"  %(self.joints[i],last[i]))
                    else:
                        velocity[i] = 0
                r.sleep()
        return True

    def active(self):
        """ Is controller overriding servo internal control? """
        return self.server.is_active() or self.executing

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if self.active():
            msg.values.append(KeyValue("State", "Active"))
        else:
            msg.values.append(KeyValue("State", "Not Active"))
        return msg

