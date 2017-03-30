#!/usr/bin/env python

import rospy, sys
import moveit_commander

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        # Connect to the right_arm move group
        left_arm = moveit_commander.MoveGroupCommander('left_arm')
        
        # Set a small tolerance on joint angles
        left_arm.set_goal_joint_tolerance(0.001)
        
        # Start the arm target in "resting" pose stored in the SRDF file
        # left_arm.set_named_target('resting')
        
        # # Plan a trajectory to the goal configuration
        # traj = left_arm.plan()
         
        # # Execute the planned trajectory
        # left_arm.execute(traj)
        
        # # Pause for a moment
        # rospy.sleep(1)
         
        # Set target joint values for the arm: joints are in the order they appear in
        # the kinematic tree.
        joint_positions = [0.15, -0.78, -1.57]
 
        # Set the arm's goal configuration to the be the joint positions
        left_arm.set_joint_value_target(joint_positions)
                 
        # Plan and execute the motion
        left_arm.go()
        rospy.sleep(5)
         
        # # Save this configuration for later
        # left_arm.remember_joint_values('saved_config', joint_positions)
         
        # # Return the arm to the named "resting" pose stored in the SRDF file
        # left_arm.set_named_target('resting')
        # left_arm.go()
        # rospy.sleep(1)
        
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
