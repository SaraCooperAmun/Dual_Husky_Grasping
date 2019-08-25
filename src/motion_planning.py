#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class MotionPlanning(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MotionPlanning, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)


    robot = moveit_commander.RobotCommander()


    scene = moveit_commander.PlanningSceneInterface()

    group_name = "left_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()

    eef_link = group.get_end_effector_link()

    group_names = robot.get_group_names()

    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    # Go to joint space configuration
    group = self.group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -1.218
    joint_goal[1] = -0.406
    joint_goal[2] = -2.405
    joint_goal[3] = -0.33
    joint_goal[4] = -0.351
    joint_goal[5] =  -1.392

    group.go(joint_goal, wait=True)


    group.stop()

  def go_to_pose_goal(self):
    # Go to pose goal
    group = self.group


    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)

    group.stop()

    group.clear_pose_targets()



  def plan_cartesian_path(self, scale=1):
    # Plan a cartesian path
    group = self.group


    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.z += scale * 0.1  
    wpose.position.y -= scale * 0.1  
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += scale * 0.2 
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold


    group.execute(plan, wait=True)

    
    return plan, fraction

  #Display trajectory in Rviz

  def display_trajectory(self, plan):

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    display_trajectory_publisher.publish(display_trajectory);


  #Executed plan computed
  def execute_plan(self, plan):

    group = self.group

    group.execute(plan, wait=True)







if __name__ == '__main__':
  try:
    motion_planning = MotionPlanning()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    motion_planning.go_to_joint_state()
    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    motion_planning.go_to_pose_goal()

    print "============ Press `Enter` to plan and compute a Cartesian path ..."
    raw_input()
    cartesian_plan, fraction = motion_planning.plan_cartesian_path()

    motion_planning.display_trajectory(cartesian_plan)
  except rospy.ROSInterruptException:
    	exit()
  except KeyboardInterrupt:
    	exit()


