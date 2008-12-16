# -*- coding: utf-8 -*-
import rostools
rostools.update_path('robot_mechanism_controllers')

from robot_mechanism_controllers.lqr_controller import *
foo=LQRProxy('left_arm_controller')
foo.cmd(['shoulder_pan_left_joint',\
      'shoulder_pitch_left_joint',\
      "upperarm_roll_left_joint",\
      "elbow_flex_left_joint",\
      "forearm_roll_left_joint",\
      "wrist_flex_left_joint",\
      "gripper_roll_left_joint"],\
    [1,0,0,0,0,0,0],\
    #[-1,-0.2,-1,-1,-1,-1,-1],\
    [0.0,0,0,0,0,0,0],\
    #[1,1,1,1,1,1,1,100,1000,100,100,100,100,100],\
    [100,1000,100,100,100,10000,100,1,1,1,1,1,1,1],\
    [0.1,0.1,0.1,0.1,0.1,0.1,0.1])
    

      #<joint name="upperarm_roll_left_joint"/>
      #<joint name="elbow_flex_left_joint"/>
      #<joint name="forearm_roll_left_joint"/>
      #<joint name="wrist_flex_left_joint"/>      
      #<joint name="gripper_roll_left_joint"/>
    
    #.013779, -5.8e-05, 0.179474
#Creating kinematic group:: left_arm
#Robot kinematics:: done creating all kinematic groups
#[DEBUG] elbow_flex_left:        4
#[DEBUG] forearm_roll_left:      5
#[DEBUG] gripper_roll_left:      7
#[DEBUG] shoulder_pan_left:      1
#[DEBUG] shoulder_pitch_left:    2
#[DEBUG] upperarm_roll_left:     3
#[DEBUG] wrist_flex_left:        6
#[DEBUG] Model has 14 states and 7 inputs, controller loaded 7 joints
#[DEBUG] DONE LOADING LQR CONTROLLER NODE
#You have to load parameters now.
#Spawned left_arm_controller