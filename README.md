# Dual_Husky_Grasping
Object grasping with Clearpath's dual-UR5 arm Husky using an AR tag and MoveIt! See demo: https://youtu.be/qf7TPQp0qIU

# Components required to run

- Husky Dual UR5 (Clearpath robotics) and installation of corresponding ROS packages: https://www.clearpathrobotics.com/assets/guides/husky/HuskyDualManip.html

- ar_track_alvar package: https://github.com/ros-perception/ar_track_alvar

- Object to grasp: used object was a 3D primet casket with a cylindrical handle. 

- AR tag (sticked to object): follow instructions here to download marker http://wiki.ros.org/ar_track_alvar


# How to run

1) Run ar_track_alvar detector
2) Run MoveIt on Husky
3) Run pick_place.py script
