#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
                
        # Initialize the move group for the right arm
        left_arm = moveit_commander.MoveGroupCommander('left_arm')
                
        # Get the name of the end-effector link
        end_effector_link = left_arm.get_end_effector_link()
                        
        # Set the reference frame for pose targets
        reference_frame = 'support_link'
        
        # Set the right arm reference frame accordingly
        left_arm.set_pose_reference_frame(reference_frame)
                
        # Allow replanning to increase the odds of a solution
        left_arm.allow_replanning(False)
        
        # Allow some leeway in position (meters) and orientation (radians)

        #left_arm.set_goal_orientation_tolerance(7)

        # Set the start state to the current state
        left_arm.set_start_state_to_current_state()

        # Start the arm in the "resting" pose stored in the SRDF file
        # left_arm.set_named_target('resting')
        # left_arm.go()
        # rospy.sleep(2)

        # Set the target pose.
        # left_arm.clear_pose_targets()  
        # target_pose = PoseStamped()
        # target_pose.header.frame_id = reference_frame
        # target_pose.header.stamp = rospy.Time.now()     
        # target_pose.pose.position.x = 0.359
        # target_pose.pose.position.y = -0.048
        # target_pose.pose.position.z = 0.330
        # target_pose.pose.orientation.w = 1.0
        
        # # # Set the goal pose of the end effector to the stored pose

        #left_arm.set_pose_target(target_pose, end_effector_link)

        left_arm.set_goal_position_tolerance(0.001)
       #left_arm.set_goal_orientation_tolerance(7)

        left_arm.set_position_target([0.359, -0.048, 0.330], end_effector_link)

  ## Then, we will get the current set of joint values for the group
        # group_variable_values = left_arm.get_current_joint_values()
        # group_variable_values[0] = 0
        # group_variable_values[1] = 0
        # group_variable_values[2] = 0
        # left_arm.set_joint_value_target(group_variable_values)

        # # Plan the trajectory to the goal
        traj = left_arm.plan()
        
        # # Execute the planned trajectory
        left_arm.execute(traj)
    
        # # Pause for a second
        rospy.sleep(1)

        # left_arm.set_start_state_to_current_state()       
        # left_arm.shift_pose_target(2, 0.1, end_effector_link)
        # left_arm.go()
        # rospy.sleep(1)
  
          
        # # Store this pose as the new target_pose
        # saved_target_pose = left_arm.get_current_pose(end_effector_link)
          
        # left_arm.set_named_target('resting')
        # left_arm.go()
        # rospy.sleep(1)
          
        # # Go back to the stored target
        # left_arm.set_pose_target(saved_target_pose, end_effector_link)
        # left_arm.go()
        # rospy.sleep(1)
           
        # # Finish up in the resting position  
        # left_arm.set_named_target('resting')
        # left_arm.go()

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()

    
    