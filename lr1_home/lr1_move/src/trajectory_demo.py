#!/usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('trajectory_demo')
        
        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)
        
        # Which joints define the arm?
        arm_joints = ['waist_left_lift_joint',
                      'left_lift_upperarm_joint',
                      'left_upperarm_forearm_joint']
        
        if reset:
            # Set the arm back to the resting position
            arm_goal  = [0, 0, 0]
        else:
            # Set a goal configuration for the arm
            arm_goal  = [0.15, -0.78, -1.57]
    
        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for right arm trajectory controller...')
        
        arm_client = actionlib.SimpleActionClient('/left_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        arm_client.wait_for_server()
        
        rospy.loginfo('...connected.')
    
        # Create a single-point arm trajectory with the arm_goal as the end-point
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
        # Send the trajectory to the arm action server
        rospy.loginfo('Moving the arm to goal position...')
        
        # Create an empty trajectory goal
        arm_goal = FollowJointTrajectoryGoal()
        
        # Set the trajectory component to the goal trajectory created above
        arm_goal.trajectory = arm_trajectory
        
        # Specify zero tolerance for the execution time
        arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # Send the goal to the action server
        arm_client.send_goal(arm_goal)
        
        arm_client.wait_for_result(rospy.Duration(10.0))

        rospy.loginfo('...done')
        
if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
    