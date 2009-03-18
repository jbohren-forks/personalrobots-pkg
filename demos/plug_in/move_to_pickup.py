#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_mechanism_controllers')
roslib.load_manifest('mechanism_control')
roslib.load_manifest('mechanism_bringup')
import numpy
import rospy

from robot_msgs.msg import JointTraj, JointTrajPoint, PlugStow, Point
from mechanism_control import mechanism
from robot_mechanism_controllers.srv import *
from pr2_mechanism_controllers.srv import *

import sys
from time import sleep

plug_found = False
found_count = 0
centroid_x =[]
centroid_y =[]
plug_location = Point()

def get_location(data):
    global found_count, centroid_x, centroid_y, plug_location
    if (data.stowed == 1):
      found_count=found_count+1
      centroid_x.append(data.plug_centroid.x)
      centroid_y.append(data.plug_centroid.y)

    if(found_count > 10):
     x=numpy.array(centroid_x)
     y=numpy.array(centroid_y)
     if (x.std()<0.1 and y.std()<0.1):
      plug_found =True
      plug_location.x = x.mean()
      plug_location.y = y.mean()
      print "found plug at x:%f y:%f\n", (plug_location.x, plug_location.y)
     
def move(positions):
 
  traj = JointTraj()
  traj.points = []
  for i in range(0,len(positions)):
    traj.points.append(JointTrajPoint())
    traj.points[i].positions = positions[i]
    traj.points[i].time = 0.0

  srv_msg = TrajectoryStart()
  srv_msg.traj = traj
  srv_msg.hastiming = 0
  srv_msg.requesttiming = 0
  
  try:
    move_arm = rospy.ServiceProxy('right_arm/trajectory_controller/TrajectoryStart', TrajectoryStart)
    resp1 = move_arm(srv_msg)
    return resp1.trajectoryid
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
  
if __name__ == '__main__':

  rospy.wait_for_service('spawn_controller')
  rospy.init_node('move_to_pickup', anonymous = True)

  # Load xml file for arm trajectory controllers
  path = roslib.packages.get_pkg_dir('sbpl_arm_executive')
  xml_for_right = open(path + '/launch/xml/r_arm_trajectory_controller.xml')

  controllers = []
  try:

    # tuck traj for left arm
    set_params()
    mechanism.spawn_controller(xml_for_right.read())
    controllers.append('right_arm/trajectory_controller')

    positions_unfold = [[-0.0118545903883, 1.40015507128, -1.49991771552, -1.94172772826, 0.0131312866653, 0.0962640845608, -0.00644695174672],
                 [-2.04879973213, 1.11987025248, -1.07494474299, -2.05521997047, 1.03679317578, 1.66637122973, 0.151969101557]]  
                 
    #positions = [positions[i] for i in range(len(positions)) if i % 1 == 0]
    #print len(positions)
    
    rospy.wait_for_service('right_arm/trajectory_controller/TrajectoryStart')

    traj_id = move(positions)
    is_traj_done = rospy.ServiceProxy('right_arm/trajectory_controller/TrajectoryQuery', TrajectoryQuery)
    
    while(is_traj_done(traj_id)!=1)
      sleep(.5)
    
    #now collect data
    
    print "collecting data" 
    rospy.Subscriber("/plug_onbase_detector_node/plug_stow_info", PlugStow, get_location)
    global plug_found
    while(!plug_found)
      sleep(.5)

    
    positions_reach = [[-1.94218984843, 1.11597858279, -0.972790862418, -2.05406188637, 1.13866187982, 1.98224613965, 0.138829401448],
                 [-0.588178005201, 1.02579336793, -1.83716985184, -2.05579901252, 2.47973068494, 1.29075854416, 0.0628192819417]]  
                 
    traj_id = move(positions)                
    while(is_traj_done(traj_id)!=1)
      sleep(.5)
    
    #now pick up plug  

    print "picking up plug"
    
    rospy.spin()
    
  finally:
    for name in controllers:
      try:
        mechanism.kill_controller(name)
      except:
        pass
