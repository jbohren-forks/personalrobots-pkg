#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_mechanism_controllers')
roslib.load_manifest('mechanism_control')
roslib.load_manifest('mechanism_bringup')
import numpy
import rospy

from robot_msgs.msg import JointTraj, JointTrajPoint, PlugStow, Point, PoseStamped
from mechanism_control import mechanism
from robot_mechanism_controllers.srv import *
from pr2_mechanism_controllers.srv import *
from std_msgs.msg import *
from robot_srvs.srv import *

import sys
from time import sleep

plug_not_found = True
found_count = 0
centroid_x =[]
centroid_y =[]
plug_location = Point()

def get_location(data):
    global found_count, centroid_x, centroid_y, plug_location, plug_not_found
    if (data.stowed == 1):
      found_count=found_count+1
      centroid_x.append(data.plug_centroid.x)
      centroid_y.append(data.plug_centroid.y)

    if(found_count >= 3):
     x=numpy.array(centroid_x)
     y=numpy.array(centroid_y)
     if (x.std()<0.1 and y.std()<0.1):
      plug_not_found = False
      plug_location.x = x.mean()
      plug_location.y = y.mean()
      print "found plug at x:%s y:%s\n", (plug_location.x, plug_location.y)

def move(positions):

  traj = JointTraj()
  traj.points = []
  for i in range(0,len(positions)):
    traj.points.append(JointTrajPoint())
    traj.points[i].positions = positions[i]
    traj.points[i].time = 0.0

  try:
    move_arm = rospy.ServiceProxy('right_arm/trajectory_controller/TrajectoryStart', TrajectoryStart)
    resp1 = move_arm(traj,0,0)
    print resp1.trajectoryid
    return resp1.trajectoryid
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def pickup():

  m0 = PoseStamped()
  m0.header.frame_id = 'torso_lift_link'#'base_link'
  m0.header.stamp = rospy.get_rostime()
  m0.pose.position.x = plug_location.x#-0.045 #0.19
  m0.pose.position.y = plug_location.y #+0.02 #0.04
  m0.pose.position.z = -0.4#0.4
  m0.pose.orientation.x = -0.19
  m0.pose.orientation.y = 0.13
  m0.pose.orientation.z = 0.68
  m0.pose.orientation.w = 0.68

  m1 = PoseStamped()
  m1.header.frame_id = 'torso_lift_link'#'base_link'
  m1.header.stamp = rospy.get_rostime()
  m1.pose.position.x = plug_location.x#-0.04 #0.19
  m1.pose.position.y = plug_location.y #+0.02 #0.04
  m1.pose.position.z = -0.55#0.20 #0.23
  m1.pose.orientation.x = -0.19
  m1.pose.orientation.y = 0.13
  m1.pose.orientation.z = 0.68
  m1.pose.orientation.w = 0.68

  print "sending pickup command"
  try:
    move_arm = rospy.ServiceProxy('cartesian_trajectory_right/move_to', MoveToPose)
    m0.header.stamp = rospy.get_rostime()
    move_arm(m0)
    m1.header.stamp = rospy.get_rostime()
    move_arm(m1)
    return
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e


def set_params():
  rospy.set_param("right_arm/trajectory_controller/velocity_scaling_factor", 0.75)
  rospy.set_param("right_arm/trajectory_controller/trajectory_wait_timeout", 0.1)

  rospy.set_param("right_arm/trajectory_controller/r_shoulder_pan_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_shoulder_lift_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_shoulder_roll_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_elbow_flex_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_forearm_roll_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_wrist_flex_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_wrist_roll_joint/goal_reached_threshold", 0.1)

  #Cartesian trajectory controller
  rospy.set_param("cartesian_trajectory_right/autostart", 'true')

  rospy.set_param("cartesian_trajectory_right/root_name", 'torso_lift_link')
  rospy.set_param("cartesian_trajectory_right/tip_name", 'r_gripper_tool_frame')

  rospy.set_param("cartesian_trajectory_right/pose/p",20.0)
  rospy.set_param("cartesian_trajectory_right/pose/i",0.1)
  rospy.set_param("cartesian_trajectory_right/pose/d",0.0)
  rospy.set_param("cartesian_trajectory_right/pose/i_clamp",0.5)

  rospy.set_param("cartesian_trajectory_right/max_vel_trans", 0.5)
  rospy.set_param("cartesian_trajectory_right/max_vel_rot", 1.0)
  rospy.set_param("cartesian_trajectory_right/max_acc_trans", 0.5)
  rospy.set_param("cartesian_trajectory_right/max_acc_rot", 0.8)


  rospy.set_param("cartesian_trajectory_right/pose/twist/ff_trans", 10.0) #20.0)
  rospy.set_param("cartesian_trajectory_right/pose/twist/ff_rot", 5.0)
  rospy.set_param("cartesian_trajectory_right/pose/twist/fb_trans/p", 20.0)
  rospy.set_param("cartesian_trajectory_right/pose/twist/fb_trans/i", 0.5)
  rospy.set_param("cartesian_trajectory_right/pose/twist/fb_trans/d", 0.0 )
  rospy.set_param("cartesian_trajectory_right/pose/twist/fb_trans/i_clamp", 1.0)
  rospy.set_param("cartesian_trajectory_right/pose/twist/fb_rot/p", 1.5 )
  rospy.set_param("cartesian_trajectory_right/pose/twist/fb_rot/i", 0.1)
  rospy.set_param("cartesian_trajectory_right/pose/twist/fb_rot/d", 0.0)
  rospy.set_param("cartesian_trajectory_right/pose/twist/fb_rot/i_clamp", 0.2)

if __name__ == '__main__':

  rospy.wait_for_service('spawn_controller')
  rospy.init_node('move_to_pickup', anonymous = True)
  rospy.wait_for_service('kill_and_spawn_controllers')
  kill_and_spawn = rospy.ServiceProxy('kill_and_spawn_controllers', KillAndSpawnControllers)

  kill_and_spawn('<controllers></controllers>', ['cartesian_trajectory_right', 'right_arm/trajectory_controller'])

  # Load xml file for arm trajectory controllers
  path = roslib.packages.get_pkg_dir('sbpl_arm_executive')
  xml_for_traj = open(path + '/launch/xml/r_arm_trajectory_controller.xml')
  path = roslib.packages.get_pkg_dir('plug_in')
  xml_for_gripper = open(path + '/gripper_controller.xml')

  pose_config = '<controller type="CartesianTrajectoryControllerNode" name="cartesian_trajectory_right"/>'


  controllers = []
  try:

    # tuck traj for left arm
    set_params()
    mechanism.spawn_controller(xml_for_gripper.read())
#    controllers.append('gripper_position_controller')
    pub = rospy.Publisher("/gripper_position_controller/set_command", Float64)
    sleep(2)
    pub.publish(Float64(0.8))
    mechanism.spawn_controller(xml_for_traj.read())
    controllers.append('right_arm/trajectory_controller')

    positions_unfold = [[-0.0118545903883, 1.40015507128, -1.49991771552, -1.94172772826, 0.0131312866653, 0.0962640845608, -0.00644695174672],
                 [-2.04879973213, 1.11987025248, -1.07494474299, -2.05521997047, 1.03679317578, 1.66637122973, 0.151969101557]]

    #positions = [positions[i] for i in range(len(positions)) if i % 1 == 0]
    #print len(positions)

    rospy.wait_for_service('right_arm/trajectory_controller/TrajectoryStart')

    traj_id = move(positions_unfold)
    is_traj_done = rospy.ServiceProxy('right_arm/trajectory_controller/TrajectoryQuery', TrajectoryQuery)
    resp =is_traj_done(traj_id)

    while(resp.done==0):
      resp=is_traj_done(traj_id)
#      print resp.done
      sleep(.5)

    #now collect data

    print "collecting data"
    rospy.Subscriber("/plug_onbase_detector_node/plug_stow_info", PlugStow, get_location)

    while(plug_not_found):
      print "plug not found\n"
      sleep(.5)

    print "picking up plug"
    positions_reach =[[-1.95437620372, 1.1029499495, -1.18591881732, -2.05724661765, 1.04622224946, 1.89800972255, 0.504874075412],
[-0.955012169549, 1.18865128408, -2.13192469091, -2.02395169966, 2.59542484058, 0.715958820011, 2.8143286509]]

    traj_id2 = move(positions_reach)
    resp2 = is_traj_done(traj_id2)
    while(resp2.done==0):
      resp2=is_traj_done(traj_id2)
      sleep(.5)
    #sys.exit(0)

    #sleep(1)
    #now pick up plug
    resp = kill_and_spawn(pose_config, ['right_arm/trajectory_controller'])
    #mechanism.kill_controller('right_arm/trajectory_controller')
    #mechanism.spawn_controller(xml_for_pose.read())
    #controllers.append('cartesian_trajectory_right')
    print "picking up plug"
    pickup()
    pub.publish(Float64(0.6))

    rospy.spin()

  finally:
    for name in controllers:
      try:
        mechanism.kill_controller(name)
      except:
        pass
