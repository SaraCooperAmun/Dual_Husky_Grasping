#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv) 
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander() 
scene = moveit_commander.PlanningSceneInterface()  
group = moveit_commander.MoveGroupCommander("left_arm") 

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)


group_variable_values = group.get_current_joint_values()

#Set the joint values for the left arm
group_variable_values[0] = -1.218
group_variable_values[1] = -0.406
group_variable_values[2] = -2.405
group_variable_values[3] = -0.33
group_variable_values[4] = -0.351
group_variable_values[5] =  -1.392


group.set_joint_value_target(group_variable_values)

group2 = moveit_commander.MoveGroupCommander("right_arm")
group_variable_values2 = group2.get_current_joint_values()


#Set the joint values for the right arm 
group_variable_values2[0] = -1.5701060894102072
group_variable_values2[1] = -1.5701060894102072
group_variable_values2[2] = -1.208566579491281
group_variable_values2[3] = -1.56990888943800
group_variable_values2[4] = 9.381241799388818e-06
group_variable_values2[5] =  9.381241799388818e-06

group2.set_joint_value_target(group_variable_values2)


#Plan the trajectories of both arms
plan = group.plan()

plan2 = group2.plan()

rospy.sleep(1)

#Execute the trajectories consecutively
group.execute(plan)

group2.execute(plan2)

moveit_commander.roscpp_shutdown()
