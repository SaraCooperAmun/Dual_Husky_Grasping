#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import roslib; roslib.load_manifest('robotiq_s_model_control')

from time import sleep
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('limpet_grasp', anonymous=True)

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('left_arm')
joints = group.get_joints()

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
gripper_publisher = rospy.Publisher("/l_gripper/SModelRobotOutput", SModelRobotOutput)

rate = rospy.Rate(1)
listener = tf.TransformListener()

trans = None
rot = None

while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/base_link', '/ar_marker_0', rospy.Time(0))
        break
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

rospy.loginfo("marker found")

#Activate and open left gripper
gcommand = SModelRobotOutput()
gcommand.rACT = 1
gcommand.rGTO = 1
gcommand.rSPA = 255
gcommand.rFRA = 150
gripper_publisher.publish(gcommand)
rospy.sleep(2)
gcommand.rPRA = 0
gripper_publisher.publish(gcommand)

#Move left arm to ready position (left to the limpet)
pose_target = geometry_msgs.msg.Pose()

pose_target.orientation.x =- 0.286

pose_target.orientation.y = 0.275

pose_target.orientation.z = -0.639

pose_target.orientation.w = 0.659
pose_target.position.x = trans[0]

pose_target.position.y = trans[1] + 0.5

pose_target.position.z = trans[2]+ 0.08


group.set_pose_target(pose_target)

plan1 = group.plan()
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory);

confirmcheck = raw_input('Press enter to continue')
if confirmcheck == "":
    group.go()
else:
    exit()
rospy.sleep(3)

while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/base_link', '/ar_marker_0', rospy.Time(0))
        break


    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass


#Pregrasp position
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x =- 0.286

pose_target.orientation.y = 0.275

pose_target.orientation.z = -0.639

pose_target.orientation.w = 0.659

pose_target.position.x = trans[0] + 0.03
pose_target.position.y = trans[1] + 0.2
pose_target.position.z = trans[2] + 0.06



group.set_pose_target(pose_target)

plan1 = group.plan()
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory);

confirmcheck = raw_input('Press enter to continue')
if confirmcheck == "":
    group.go()
else:
    exit()
rospy.sleep(3)

#Close gripper
gcommand = SModelRobotOutput()
gcommand.rACT = 1
gcommand.rGTO = 1
gcommand.rSPA = 255
gcommand.rFRA = 150
gripper_publisher.publish(gcommand)
rospy.sleep(1)
gcommand.rPRA = 255
gripper_publisher.publish(gcommand)

rospy.sleep(2)

#Retreat arm with limpet grasped

pose_target = geometry_msgs.msg.Pose()

pose_target.orientation.x = -0.286

pose_target.orientation.y = 0.275

pose_target.orientation.z = -0.639

pose_target.orientation.w = 0.659

pose_target.position.x = trans[0] - 0.3

pose_target.position.y = trans[1] +  0.4

pose_target.position.z = trans[2] + 0.1


group.set_pose_target(pose_target)

plan1 = group.plan()
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory);

confirmcheck = raw_input('Press enter to continue')
if confirmcheck == "":
    group.go()
else:
    exit()

rospy.sleep(3)

#Begin place operation
while not rospy.is_shutdown():
    try: 
         (trans2,rot2) = listener.lookupTransform('/base_link', '/ar_marker_2', rospy.Time(0))
         break
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
         pass
  #Pre-place position
  pose_target = geometry_msgs.msg.Pose()

  pose_target.orientation.x = -0.286

  pose_target.orientation.y = 0.275

  pose_target.orientation.z = -0.639

  pose_target.orientation.w = 0.659

  pose_target.position.x = trans2[0] - 0.15  #Offsets require further adjustment

  pose_target.position.y = trans2[1]+0.2 

  pose_target.position.z = trans2[2] + 0.1

  #Place position. As the second marker is blocked it won't be detected
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.x =- 0.286

  pose_target.orientation.y = 0.275

  pose_target.orientation.z = -0.639

  pose_target.orientation.w = 0.659

  pose_target.position.x = trans2[0] -0.15
  pose_target.position.y = trans2[1] 
  pose_target.position.z = trans2[2] 

  group.set_pose_target(pose_target)
  plan1 = group.plan()
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);

  confirmcheck = raw_input('Press enter to continue')
  if confirmcheck == "":
        group.go()
  else:
        exit()
  #Open and place limpet

  group.set_pose_target(pose_target)
  gcommand = SModelRobotOutput()
  gcommand.rACT = 1 
  gcommand.rGTO = 1
  gcommand.rSPA = 255
  gcommand.rFRA = 150
  gripper_publisher.publish(gcommand)
  rospy.sleep(1)
  gcommand.rPRA =0
  gripper_publisher.publish(gcommand)
  #Retreat arm, taking into account ar_marker_0, placed on top of limpet
  while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/base_link', '/ar_marker_0', rospy.Time(0))
        break


    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
  pose_target = geometry_msgs.msg.Pose()

  pose_target.orientation.x = -0.286

  pose_target.orientation.y = 0.275

  pose_target.orientation.z = -0.639

  pose_target.orientation.w = 0.659

  pose_target.position.x = trans[0] - 0.3

  pose_target.position.y = trans[1] +  0.4

  pose_target.position.z = trans[2] + 0.1


  group.set_pose_target(pose_target)

  plan1 = group.plan()
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);

  confirmcheck = raw_input('Press enter to continue')
  if confirmcheck == "":
    group.go()
  else:
    exit()

exit()









