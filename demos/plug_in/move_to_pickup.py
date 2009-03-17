#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_mechanism_controllers')
roslib.load_manifest('mechanism_control')
roslib.load_manifest('mechanism_bringup')

import rospy

from robot_msgs.msg import JointTraj, JointTrajPoint
from mechanism_control import mechanism
from robot_mechanism_controllers.srv import *

import sys
from time import sleep

def move(positions):
  pub = rospy.Publisher('r_arm/trajectory_controller/trajectory_command', JointTraj)

  # HACK
  sleep(2)

  msg = JointTraj()
  msg.points = []
  for i in range(0,len(positions)):
    msg.points.append(JointTrajPoint())
    msg.points[i].positions = positions[i]
    msg.points[i].time = 0.0

  pub.publish(msg)
  
def set_params():
  rospy.set_param("right_arm/trajectory_controller/velocity_scaling_factor", 0.75)
  rospy.set_param("right_arm/trajectory_controller/trajectory_wait_timeout", 0.25)

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

  positions = [ [-0.0169115133318 1.4054003652 -1.49799330961 -2.04653433969 -0.0319894340348 0.0969602276129 -0.00707822692357]
                [-0.0169115133318 1.4054003652 -1.49991771552 -2.05160095765 -0.0319894340348 0.0994402372362 -0.00955823654684]
                [-0.0169115133318 1.4054003652 -1.49959698121 -2.05189047867 -0.0319894340348 0.263338417076 -0.0373604496919]
                [-0.029097868622 1.4054003652 -1.47923035196 -2.05435140739 -0.0319894340348 0.564420287129 -0.0205659985589]
                [-0.206753374655 1.4054003652 -1.48885238152 -2.05435140739 -0.0319894340348 0.859845994883 -0.0192607303361]
                [-0.428843482291 1.4054003652 -1.55668768997 -2.05449616791 -0.0319894340348 1.07230015261 -0.0281800631917]
                [-0.630042695144 1.40531576369 -1.57480917899 -2.05435140739 -0.0319894340348 1.18542339858 -0.0402755487228]
                [-0.802475477482 1.40523116217 -1.57208293728 -2.05362760483 -0.0319894340348 1.53597493428 -0.0869606421573]
                [-0.883552045331 1.40523116217 -1.53102894446 -2.05377236534 -0.0319894340348 1.81652058429 0.155645211515]
                [-1.04114566034 1.40523116217 -1.42037560444 -2.05406188637 -0.0319894340348 1.91715676427 0.176224940494]
                [-1.35666449187 1.40523116217 -1.42117744023 -2.05261428124 -0.0319894340348 1.91946273813 0.178530914355]
                [-1.63214244173 1.40506195914 -1.4205359716 -2.05116667611 0.0455834973227 1.91472026359 0.183621460424]
                [-1.74654496078 1.40506195914 -1.30988263158 -2.05116667611 0.771853559361 1.82383008634 0.274511637669]
                [-1.74298024461 1.40514656066 -1.28823306505 -2.0533380838 1.35500995087 1.82383008634 0.274511637669]
                [-1.74273154348 1.4065001849 -1.3196650283 -2.05290380226 1.4806538039 1.74921225294 0.199893804267] 
                [-1.7423170416 1.40336992885 -1.31838209103 -2.05391712585 1.48152151007 1.77562217998 0.173483877227]
                [-1.74223414122 1.40336992885 -1.31838209103 -2.05507520996 1.48152151007 1.77562217998 0.173483877227]
                [-1.70119845504 1.39956286068 -1.45597711384 -2.06694557203 1.70243950026 1.78732608504 0.17370142193]
                [-1.58555243035 1.40523116217 -1.61730647624 -2.06766937459 2.00873977732 1.77910289524 -0.21035199815]
                [-1.51972953171 1.40836141822 -1.99641444118 -2.05782565971 2.36918491922 1.55085499202 -0.819129097251]
                [-1.33527619483 1.40353913188 -2.05414661858 -1.9654684524 2.57506266919 1.34105487968 -0.73019682234]
                [-1.28528726803 1.38171194104 -2.1216611927 -1.88758729639 2.62521608566 1.2872778289 -0.676506789443]
                [-1.30294504814 1.35269362143 -2.19430751593 -1.8765854974 2.65708982554 1.20326206429 -0.609372493852]
                [-1.30294504814 1.32257548213 -2.22798461942 -1.87629597638 2.74773619648 1.13234249086 -0.54393504695]
                [-1.30145284137 1.30311713371 -2.25316226345 -1.90220810821 2.74912452635 1.13208143721 -0.53305781176] ]  
  move(positions)

  rospy.spin()
  
finally:
  for name in controllers:
    try:
      mechanism.kill_controller(name)
    except:
      pass
