/*
 *  rosgazebo
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <RosGazeboNode/RosGazeboNode.h>
#include <iostream>
#include <sstream>

void
RosGazeboNode::cmd_rightarmconfigReceived()
{
  this->lock.lock();
  newRightArmPos = true;
  /*
  printf("turret angle: %.3f\n", this->rightarm.turretAngle);
  printf("shoulder pitch : %.3f\n", this->rightarm.shoulderLiftAngle);
  printf("shoulder roll: %.3f\n", this->rightarm.upperarmRollAngle);
  printf("elbow pitch: %.3f\n", this->rightarm.elbowAngle);
  printf("elbow roll: %.3f\n", this->rightarm.forearmRollAngle);
  printf("wrist pitch angle: %.3f\n", this->rightarm.wristPitchAngle);
  printf("wrist roll: %.3f\n", this->rightarm.wristRollAngle);
  printf("gripper gap: %.3f\n", this->rightarm.gripperGapCmd);
  
  double jointPosition[] = {this->rightarm.turretAngle,
                            this->rightarm.shoulderLiftAngle,
                            this->rightarm.upperarmRollAngle,
                            this->rightarm.elbowAngle,
                            this->rightarm.forearmRollAngle,
                            this->rightarm.wristPitchAngle,
                            this->rightarm.wristRollAngle,
                            this->rightarm.gripperGapCmd};
  double jointSpeed[] = {0,0,0,0,0,0,0,0};

  //  this->PR2Copy->SetArmJointPosition(PR2::PR2_LEFT_ARM, jointPosition, jointSpeed);
  */

  printf("boo!\n");
  if(!useControllerArray){
    printf("hoo!\n");
    
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_R_PAN           , this->rightarm.turretAngle,       0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_PITCH, this->rightarm.shoulderLiftAngle, 0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_ROLL , this->rightarm.upperarmRollAngle, 0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_PITCH   , this->rightarm.elbowAngle,        0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_ROLL    , this->rightarm.forearmRollAngle,  0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_R_WRIST_PITCH   , this->rightarm.wristPitchAngle,   0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_R_WRIST_ROLL    , this->rightarm.wristRollAngle,    0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_R_GRIPPER_GAP   , this->rightarm.gripperGapCmd,     0);
  }

  this->lock.unlock();
}


void
RosGazeboNode::cmd_leftarmconfigReceived()
{
  this->lock.lock();
  newLeftArmPos = true;
  //printf("Left arm command received\n");
  /*
  double jointPosition[] = {this->leftarm.turretAngle,
                            this->leftarm.shoulderLiftAngle,
                            this->leftarm.upperarmRollAngle,
                            this->leftarm.elbowAngle,
                            this->leftarm.forearmRollAngle,
                            this->leftarm.wristPitchAngle,
                            this->leftarm.wristRollAngle,
                            this->leftarm.gripperGapCmd};
  double jointSpeed[] = {0,0,0,0,0,0,0,0};
  this->PR2Copy->SetArmJointPosition(PR2::PR2_LEFT_ARM, jointPosition, jointSpeed);
  */

  if(!useControllerArray){
    
    
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_L_PAN           , this->leftarm.turretAngle,       0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_L_SHOULDER_PITCH, this->leftarm.shoulderLiftAngle, 0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_L_SHOULDER_ROLL , this->leftarm.upperarmRollAngle, 0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_L_ELBOW_PITCH   , this->leftarm.elbowAngle,        0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_L_ELBOW_ROLL    , this->leftarm.forearmRollAngle,  0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_L_WRIST_PITCH   , this->leftarm.wristPitchAngle,   0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_L_WRIST_ROLL    , this->leftarm.wristRollAngle,    0);
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_L_GRIPPER_GAP   , this->leftarm.gripperGapCmd,     0);
  }

  this->lock.unlock();
}

void RosGazeboNode::cmd_leftarmcartesianReceived() {
  this->lock.lock();

  KDL::Frame f;
  for(int i = 0; i < 9; i++) {
    f.M.data[i] = cmd_leftarmcartesian.rot[i];
  }
  for(int i = 0; i < 3; i++) {
    f.p.data[i] = cmd_leftarmcartesian.trans[i];
  }

	bool reachable;
  this->PR2Copy->SetArmCartesianPosition(PR2::PR2_LEFT_ARM,f, reachable);
  this->lock.unlock();
}

void RosGazeboNode::cmd_rightarmcartesianReceived() {
  this->lock.lock();

  KDL::Frame f;
  for(int i = 0; i < 9; i++) {
    f.M.data[i] = cmd_rightarmcartesian.rot[i];
  }
  for(int i = 0; i < 3; i++) {
    f.p.data[i] = cmd_rightarmcartesian.trans[i];
  }

	bool reachable;
  this->PR2Copy->SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f, reachable);
  this->lock.unlock();
}

bool RosGazeboNode::SetRightArmCartesian(rosgazebo::MoveCartesian::request &req, rosgazebo::MoveCartesian::response &res)
{
  this->lock.lock();
  KDL::Frame f;
  for(int i = 0; i < 9; i++)
    f.M.data[i] = req.e.rot[i];

  for(int i = 0; i < 3; i++)
    f.p.data[i] = req.e.trans[i];

	bool reachable;
  this->PR2Copy->SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f,reachable);
	res.reachable = (reachable==false) ? -1 : 0;

  this->lock.unlock();
	return true;
}

bool RosGazeboNode::OperateRightGripper(rosgazebo::GripperCmd::request &req, rosgazebo::GripperCmd::response &res)
{
	this->lock.lock();
	this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_R_GRIPPER_GAP, req.gap, 0);
	this->lock.unlock();
	return true;
}

bool RosGazeboNode::reset_IK_guess(rosgazebo::VoidVoid::request &req, rosgazebo::VoidVoid::response &res)
{
  this->lock.lock();
	this->PR2Copy->GetArmJointPositionCmd(PR2::PR2_RIGHT_ARM, *(this->PR2Copy->right_arm_chain_->q_IK_guess));
  this->lock.unlock();
	return true;
}

void
RosGazeboNode::cmdvelReceived()
{
  this->lock.lock();
  double dt;
  double w11, w21, w12, w22;

  // smooth out the commands by time decay
  // with w1,w2=1, this means equal weighting for new command every second
  this->PR2Copy->GetTime(&(this->simTime));
  dt = simTime - lastTime;

  // smooth if dt is larger than zero
  if (dt > 0.0)
  {
    w11 =  1.0;
    w21 =  1.0;
    w12 =  1.0;
    w22 =  1.0;
    this->vxSmooth = (w11 * this->vxSmooth + w21*dt *this->velMsg.vx)/( w11 + w21*dt);
    this->vwSmooth = (w12 * this->vwSmooth + w22*dt *this->velMsg.vw)/( w12 + w22*dt);
  }

  // when running with the 2dnav stack, we need to refrain from moving when steering angles are off.
  // when operating with the keyboard, we need instantaneous setting of both velocity and angular velocity.

  // std::cout << "received cmd: vx " << this->velMsg.vx << " vw " <<  this->velMsg.vw
  //           << " vxsmoo " << this->vxSmooth << " vxsmoo " <<  this->vwSmooth
  //           << " | steer erros: " << this->PR2Copy->BaseSteeringAngleError() << " - " <<  M_PI/100.0
  //           << std::endl;

  // 2dnav: if steering angle is wrong, don't move or move slowly
  if (this->PR2Copy->BaseSteeringAngleError() > M_PI/100.0)
  {
    // set steering angle only
    this->PR2Copy->SetBaseSteeringAngle    (this->vxSmooth,0.0,this->vwSmooth);
  }
  else
  {
    // set base velocity
    this->PR2Copy->SetBaseCartesianSpeedCmd(this->vxSmooth, 0.0, this->vwSmooth);
  }

  // TODO: this is a hack, need to rewrite
  //       if we are trying to stop, send the command through
  if (this->velMsg.vx == 0.0)
  {
    // set base velocity
    this->PR2Copy->SetBaseCartesianSpeedCmd(this->vxSmooth, 0.0, this->vwSmooth);
  }

  this->lastTime = this->simTime;

  this->lock.unlock();
}

RosGazeboNode::RosGazeboNode(int argc, char** argv, const char* fname,
         PR2::PR2Robot          *myPR2 ) :
        ros::node("rosgazebo"),tf(*this),tfc(*this)
{
  // accept passed in robot
  this->PR2Copy = myPR2;

  // initialize random seed
  srand(time(NULL));

  // initialize times
  this->PR2Copy->GetTime(&(this->lastTime));
  this->PR2Copy->GetTime(&(this->simTime));

  //No new messages
  newRightArmPos = false;
  newLeftArmPos = false;

  //Don't use new architecture
  useControllerArray = false;
} 

RosGazeboNode::RosGazeboNode(int argc, char** argv, const char* fname,
         PR2::PR2Robot          *myPR2,
         controller::ArmController          *myArm,
         controller::HeadController         *myHead,
         controller::SpineController        *mySpine,
         controller::BaseController         *myBase,
         controller::LaserScannerController *myLaserScanner,
         controller::GripperController      *myGripper) :
        ros::node("rosgazebo"),tf(*this),tfc(*this)
{
  // accept passed in robot
  this->PR2Copy = myPR2;

  // initialize random seed
  srand(time(NULL));

  // initialize times
  this->PR2Copy->GetTime(&(this->lastTime));
  this->PR2Copy->GetTime(&(this->simTime));

  //No new messages
  newRightArmPos = false;
  newLeftArmPos = false;

  //Don't use new architecture
  useControllerArray = false;
  
  //TODO: if you want to advertise some information about joints, this is the place to do it:
  //   JointController * controller = ...
#ifdef EXPOSE_JOINTS
  
  for(int i=0;i<110-75;i++)
  {
    std::ostringstream ss;
    ss<<"JOINT_"<<i;
    controller::RosJoint * rjoint;
    rjoint = new controller::RosJoint(&(PR2Copy->hw),PR2::ARM_L_PAN,ss.str());
    RosJoints.push_back(rjoint);
  }
//   RosJoints.push_back(rjoint);
//   rjoint = new controller::RosJoint(&(PR2Copy->hw),PR2::ARM_L_ELBOW_PITCH,"ARM_L_ELBOW_PITCH");
  
#endif
  
  //   RosControllers.push_back(new RosJointController(controller))
//   for(mechanism::Robot::IndexMap::iterator it=PR2Copy->joints_lookup_.begin() ; it!=PR2Copy->joints_lookup_.begin() ; ++it)
//   {
//     std::cout<<(*it).first<<std::endl;
//   }
}

RosGazeboNode::RosGazeboNode(int argc, char** argv, const char* fname,
         PR2::PR2Robot          *myPR2,
         controller::ArmController          *myArm,
         controller::HeadController         *myHead,
         controller::SpineController        *mySpine,
         controller::BaseController         *myBase,
         controller::LaserScannerController *myLaserScanner,
         controller::GripperController      *myGripper,
         controller::JointController** ControllerArray,
         controller::RosJointController ** RosControllerArray):
        ros::node("rosgazebo"),tf(*this),tfc(*this)
{
  // accept passed in robot
  this->PR2Copy = myPR2;

  //Store copy of Controller Array. Only interact with it during Update() calls.
  this->ControllerArray = ControllerArray;
  // initialize random seed
  srand(time(NULL));

  // initialize times
  this->PR2Copy->GetTime(&(this->lastTime));
  this->PR2Copy->GetTime(&(this->simTime));

  //No new messages
  newRightArmPos = false;
  newLeftArmPos = false;

  //Use new architecture
  useControllerArray = true;
  
      //Store copy of Controller Array. Only interact with it during Update() calls.
  for(int i=0; i<PR2::MAX_JOINTS; ++i)
    if(RosControllerArray[i] && RosControllerArray[i]->jc && RosControllerArray[i]->jc->getName() != "")
    {
      std::cout<<"Adding "<<RosControllerArray[i]->jc->getName()<<std::endl;
      RosControllers.push_back(RosControllerArray[i]);
    }
      
}

void
RosGazeboNode::LoadRobotModel()
{
  // matches send.xml
  std::string pr2Content;
  get_param("robotdesc/pr2",pr2Content);

  // parse the big pr2.xml string from ros
  pr2Description.loadString(pr2Content.c_str());

  // get all the parameters needed for frame transforms
  link = pr2Description.getLink("base");
  base_center_offset_z = link->collision->xyz[2];
  link = pr2Description.getLink("torso");
  base_torso_offset_x  = link->xyz[0];
  base_torso_offset_y  = link->xyz[1];
  base_torso_offset_z  = link->xyz[2];
  link = pr2Description.getLink("shoulder_pan_left");
  sh_pan_left_torso_offset_x =  link->xyz[0];
  sh_pan_left_torso_offset_y =  link->xyz[1];
  sh_pan_left_torso_offset_z =  link->xyz[2];
  link = pr2Description.getLink("shoulder_pitch_left");
  shoulder_pitch_left_offset_x = link->xyz[0];
  shoulder_pitch_left_offset_y = link->xyz[1];
  shoulder_pitch_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("upperarm_roll_left");
  upperarm_roll_left_offset_x = link->xyz[0];
  upperarm_roll_left_offset_y = link->xyz[1];
  upperarm_roll_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("elbow_flex_left");
  elbow_flex_left_offset_x = link->xyz[0];
  elbow_flex_left_offset_y = link->xyz[1];
  elbow_flex_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("finger_l_left");
  finger_l_left_offset_x = link->xyz[0];
  finger_l_left_offset_y = link->xyz[1];
  finger_l_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("forearm_roll_left");
  forearm_roll_left_offset_x = link->xyz[0];
  forearm_roll_left_offset_y = link->xyz[1];
  forearm_roll_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("wrist_flex_left");
  wrist_flex_left_offset_x = link->xyz[0];
  wrist_flex_left_offset_y = link->xyz[1];
  wrist_flex_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("gripper_roll_left");
  gripper_roll_left_offset_x = link->xyz[0];
  gripper_roll_left_offset_y = link->xyz[1];
  gripper_roll_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("finger_r_left");
  finger_r_left_offset_x = link->xyz[0];
  finger_r_left_offset_y = link->xyz[1];
  finger_r_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("finger_tip_l_left");
  finger_tip_l_left_offset_x = link->xyz[0];
  finger_tip_l_left_offset_y = link->xyz[1];
  finger_tip_l_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("finger_tip_r_left");
  finger_tip_r_left_offset_x = link->xyz[0];
  finger_tip_r_left_offset_y = link->xyz[1];
  finger_tip_r_left_offset_z = link->xyz[2];


  link = pr2Description.getLink("shoulder_pan_right");
  shoulder_pan_right_offset_x = link->xyz[0];
  shoulder_pan_right_offset_y = link->xyz[1];
  shoulder_pan_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("shoulder_pitch_right");
  shoulder_pitch_right_offset_x = link->xyz[0];
  shoulder_pitch_right_offset_y = link->xyz[1];
  shoulder_pitch_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("upperarm_roll_right");
  upperarm_roll_right_offset_x = link->xyz[0];
  upperarm_roll_right_offset_y = link->xyz[1];
  upperarm_roll_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("elbow_flex_right");
  elbow_flex_right_offset_x = link->xyz[0];
  elbow_flex_right_offset_y = link->xyz[1];
  elbow_flex_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("forearm_roll_right");
  forearm_roll_right_offset_x = link->xyz[0];
  forearm_roll_right_offset_y = link->xyz[1];
  forearm_roll_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("wrist_flex_right");
  wrist_flex_right_offset_x = link->xyz[0];
  wrist_flex_right_offset_y = link->xyz[1];
  wrist_flex_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("gripper_roll_right");
  gripper_roll_right_offset_x = link->xyz[0];
  gripper_roll_right_offset_y = link->xyz[1];
  gripper_roll_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("finger_l_right");
  finger_l_right_offset_x = link->xyz[0];
  finger_l_right_offset_y = link->xyz[1];
  finger_l_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("finger_r_right");
  finger_r_right_offset_x = link->xyz[0];
  finger_r_right_offset_y = link->xyz[1];
  finger_r_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("finger_tip_l_right");
  finger_tip_l_right_offset_x = link->xyz[0];
  finger_tip_l_right_offset_y = link->xyz[1];
  finger_tip_l_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("finger_tip_r_right");
  finger_tip_r_right_offset_x = link->xyz[0];
  finger_tip_r_right_offset_y = link->xyz[1];
  finger_tip_r_right_offset_z = link->xyz[2];

  link = pr2Description.getLink("forearm_camera_left");
  forearm_camera_left_offset_x = link->xyz[0];
  forearm_camera_left_offset_y = link->xyz[1];
  forearm_camera_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("forearm_camera_right");
  forearm_camera_right_offset_x = link->xyz[0];
  forearm_camera_right_offset_y = link->xyz[1];
  forearm_camera_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("wrist_camera_left");
  wrist_camera_left_offset_x = link->xyz[0];
  wrist_camera_left_offset_y = link->xyz[1];
  wrist_camera_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("wrist_camera_right");
  wrist_camera_right_offset_x = link->xyz[0];
  wrist_camera_right_offset_y = link->xyz[1];
  wrist_camera_right_offset_z = link->xyz[2];


  link = pr2Description.getLink("head_pan");
  head_pan_offset_x = link->xyz[0];
  head_pan_offset_y = link->xyz[1];
  head_pan_offset_z = link->xyz[2];
  link = pr2Description.getLink("head_tilt");
  head_tilt_offset_x = link->xyz[0];
  head_tilt_offset_y = link->xyz[1];
  head_tilt_offset_z = link->xyz[2];
  link = pr2Description.getLink("base_laser");
  base_laser_offset_x = link->xyz[0];
  base_laser_offset_y = link->xyz[1];
  base_laser_offset_z = link->xyz[2];
  link = pr2Description.getLink("tilt_laser");
  tilt_laser_offset_x = link->xyz[0];
  tilt_laser_offset_y = link->xyz[1];
  tilt_laser_offset_z = link->xyz[2],
  link = pr2Description.getLink("caster_front_left");
  caster_front_left_offset_x = link->xyz[0];
  caster_front_left_offset_y = link->xyz[1];
  caster_front_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("wheel_front_left_l");
  wheel_front_left_l_offset_x = link->xyz[0];
  wheel_front_left_l_offset_y = link->xyz[1];
  wheel_front_left_l_offset_z = link->xyz[2];
  link = pr2Description.getLink("wheel_front_left_r");
  wheel_front_left_r_offset_x = link->xyz[0];
  wheel_front_left_r_offset_y = link->xyz[1];
  wheel_front_left_r_offset_z = link->xyz[2];
  link = pr2Description.getLink("caster_front_right");
  caster_front_right_offset_x = link->xyz[0];
  caster_front_right_offset_y = link->xyz[1];
  caster_front_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("wheel_front_right_l");
  wheel_front_right_l_offset_x = link->xyz[0];
  wheel_front_right_l_offset_y = link->xyz[1];
  wheel_front_right_l_offset_z = link->xyz[2];
  link = pr2Description.getLink("wheel_front_right_r");
  wheel_front_right_r_offset_x = link->xyz[0];
  wheel_front_right_r_offset_y = link->xyz[1];
  wheel_front_right_r_offset_z = link->xyz[2];
  link = pr2Description.getLink("caster_rear_left");
  caster_rear_left_offset_x = link->xyz[0];
  caster_rear_left_offset_y = link->xyz[1];
  caster_rear_left_offset_z = link->xyz[2];
  link = pr2Description.getLink("wheel_rear_left_l");
  wheel_rear_left_l_offset_x = link->xyz[0];
  wheel_rear_left_l_offset_y = link->xyz[1];
  wheel_rear_left_l_offset_z = link->xyz[2];
  link = pr2Description.getLink("wheel_rear_left_r");
  wheel_rear_left_r_offset_x = link->xyz[0];
  wheel_rear_left_r_offset_y = link->xyz[1];
  wheel_rear_left_r_offset_z = link->xyz[2];
  link = pr2Description.getLink("caster_rear_right");
  caster_rear_right_offset_x = link->xyz[0];
  caster_rear_right_offset_y = link->xyz[1];
  caster_rear_right_offset_z = link->xyz[2];
  link = pr2Description.getLink("wheel_rear_right_l");
  wheel_rear_right_l_offset_x = link->xyz[0];
  wheel_rear_right_l_offset_y = link->xyz[1];
  wheel_rear_right_l_offset_z = link->xyz[2];
  link = pr2Description.getLink("wheel_rear_right_r");
  wheel_rear_right_r_offset_x = link->xyz[0];
  wheel_rear_right_r_offset_y = link->xyz[1];
  wheel_rear_right_r_offset_z = link->xyz[2];


}




int
RosGazeboNode::AdvertiseSubscribeMessages()
{
  advertise<std_msgs::PointCloudFloat32>("cloudStereo");

  advertise<std_msgs::RobotBase2DOdom>("odom");
  advertise<std_msgs::PR2Arm>("left_pr2arm_pos");
  advertise<std_msgs::PR2Arm>("right_pr2arm_pos");
  advertise<rostools::Time>("time");
  advertise<std_msgs::Empty>("transform");
  advertise<std_msgs::Point3DFloat32>("object_position");

  subscribe("cmd_vel", velMsg, &RosGazeboNode::cmdvelReceived);
  subscribe("cmd_leftarmconfig", leftarm, &RosGazeboNode::cmd_leftarmconfigReceived);
  subscribe("cmd_rightarmconfig", rightarm, &RosGazeboNode::cmd_rightarmconfigReceived);
  subscribe("cmd_leftarm_cartesian", cmd_leftarmcartesian, &RosGazeboNode::cmd_leftarmcartesianReceived);
  subscribe("cmd_rightarm_cartesian", cmd_rightarmcartesian, &RosGazeboNode::cmd_rightarmcartesianReceived);
  
  for(RCList::iterator it = RosControllers.begin(); it != RosControllers.end(); ++it)
  {
    (*it)->AdvertiseSubscribeMessages();
  }

  for(RJList::iterator it = RosJoints.begin(); it != RosJoints.end(); ++it)
  {
    (*it)->AdvertiseSubscribeMessages();
  }	//------ services ----------
  advertise_service("reset_IK_guess", &RosGazeboNode::reset_IK_guess);
  advertise_service("operate_right_gripper", &RosGazeboNode::OperateRightGripper);
  advertise_service("set_rightarm_cartesian", &RosGazeboNode::SetRightArmCartesian);

  return(0);
}

RosGazeboNode::~RosGazeboNode()
{
}

double
RosGazeboNode::GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

void
RosGazeboNode::UpdateRightArm(){
  ControllerArray[PR2::ARM_R_PAN]->setPosCmd(this->rightarm.turretAngle); 
  ControllerArray[PR2::ARM_R_SHOULDER_PITCH]->setPosCmd(this->rightarm.shoulderLiftAngle);
  ControllerArray[PR2::ARM_R_SHOULDER_ROLL]->setPosCmd(this->rightarm.upperarmRollAngle);
  ControllerArray[PR2::ARM_R_ELBOW_PITCH]->setPosCmd(this->rightarm.elbowAngle);
  ControllerArray[PR2::ARM_R_ELBOW_ROLL]->setPosCmd(this->rightarm.forearmRollAngle);
  ControllerArray[PR2::ARM_R_WRIST_PITCH]->setPosCmd(this->rightarm.wristPitchAngle);
  ControllerArray[PR2::ARM_R_WRIST_ROLL]->setPosCmd(this->rightarm.wristRollAngle);

  //Mark that we've consumed the right arm message
  newRightArmPos=false; 
}

void
RosGazeboNode::UpdateLeftArm(){
  ControllerArray[PR2::ARM_L_PAN]->setPosCmd(this->leftarm.turretAngle); 
  ControllerArray[PR2::ARM_L_SHOULDER_PITCH]->setPosCmd(this->leftarm.shoulderLiftAngle);
  ControllerArray[PR2::ARM_L_SHOULDER_ROLL]->setPosCmd(this->leftarm.upperarmRollAngle);
  ControllerArray[PR2::ARM_L_ELBOW_PITCH]->setPosCmd(this->leftarm.elbowAngle);
  ControllerArray[PR2::ARM_L_ELBOW_ROLL]->setPosCmd(this->leftarm.forearmRollAngle);
  ControllerArray[PR2::ARM_L_WRIST_PITCH]->setPosCmd(this->leftarm.wristPitchAngle);
  ControllerArray[PR2::ARM_L_WRIST_ROLL]->setPosCmd(this->leftarm.wristRollAngle);

  //Mark that we've consumed the left arm message
  newLeftArmPos = false;
}

void
RosGazeboNode::Update()
{
  this->lock.lock();

  /***************************************************************/
  /*                                                             */
  /*  publish time                                               */
  /*                                                             */
  /***************************************************************/
  this->PR2Copy->GetTime(&(this->simTime));
  timeMsg.rostime.sec  = (unsigned long)floor(this->simTime);
  timeMsg.rostime.nsec = (unsigned long)floor(  1e9 * (  this->simTime - timeMsg.rostime.sec) );
  publish("time",timeMsg);


  /***************************************************************/
  /*                                                             */
  /*  Arm Updates                                                */
  /*                                                             */
  /***************************************************************/
  if(useControllerArray){
    if(newLeftArmPos)UpdateLeftArm();
    if(newRightArmPos)UpdateRightArm();
  }

  /***************************************************************/
  /*                                                             */
  /*  laser - pitching                                           */
  /*                                                             */
  /***************************************************************/

  /***************************************************************/
  /*                                                             */
  /*  laser - base                                               */
  /*                                                             */
  /***************************************************************/

  /***************************************************************/
  /*                                                             */
  /*  odometry                                                   */
  /*                                                             */
  /***************************************************************/
  // Get latest odometry data
  // Get velocities
  double vx,vy,vw;
  this->PR2Copy->GetBaseCartesianSpeedActual(&vx,&vy,&vw);
  // Translate into ROS message format and publish
  this->odomMsg.vel.x  = vx;
  this->odomMsg.vel.y  = vy;
  this->odomMsg.vel.th = vw;

  // Get position
  double x,y,z,roll,pitch,yaw;
  this->PR2Copy->GetBasePositionActual(&x,&y,&z,&roll,&pitch,&yaw);
  this->odomMsg.pos.x  = x;
  this->odomMsg.pos.y  = y;
  this->odomMsg.pos.th = yaw;

  // this->odomMsg.stall = this->positionmodel->Stall();

  // TODO: get the frame ID from somewhere
  this->odomMsg.header.frame_id = tf.lookup("FRAMEID_ODOM");

  this->odomMsg.header.stamp.sec = (unsigned long)floor(this->simTime);
  this->odomMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->simTime - this->odomMsg.header.stamp.sec) );

  // This publish call resets odomMsg.header.stamp.sec and 
  // odomMsg.header.stamp.nsec to zero.  Thus, it must be called *after*
  // those values are reused in the sendInverseEuler() call above.
  publish("odom",this->odomMsg);

  /***************************************************************/
  /*                                                             */
  /*   object position                                           */
  /*                                                             */
  /***************************************************************/
  this->PR2Copy->GetObjectPositionActual(&x,&y,&z,&roll,&pitch,&yaw);
  this->objectPosMsg.x  = x;
  this->objectPosMsg.y  = y;
  this->objectPosMsg.z  = z;
  publish("object_position", this->objectPosMsg);

  /***************************************************************/
  /*                                                             */
  /*  camera                                                     */
  /*                                                             */
  /***************************************************************/
  // deprecated to using ros+gazebo plugins


  /***************************************************************/
  /*                                                             */
  /*  pitching Hokuyo joint                                      */
  /*                                                             */
  /***************************************************************/
  static double dAngle = -1;
  double simPitchFreq,simPitchAngle,simPitchRate,simPitchTimeScale,simPitchAmp,simPitchOffset;
  simPitchFreq      = 1.0/60.0;
  simPitchTimeScale = 2.0*M_PI*simPitchFreq;
  simPitchAmp    =  M_PI / 10.0;
  simPitchOffset = -M_PI / 10.0;
  simPitchAngle = simPitchOffset + simPitchAmp * sin(this->simTime * simPitchTimeScale);
  simPitchRate  =  simPitchAmp * simPitchTimeScale * cos(this->simTime * simPitchTimeScale); // TODO: check rate correctness
  this->PR2Copy->GetTime(&this->simTime);
  this->PR2Copy->hw.SetJointTorque(PR2::HEAD_LASER_PITCH , 200.0);
  this->PR2Copy->hw.SetJointGains(PR2::HEAD_LASER_PITCH, 3.0, 1.0, 0.0);
  this->PR2Copy->hw.SetJointServoCmd(PR2::HEAD_LASER_PITCH , -simPitchAngle, simPitchRate);

  if (dAngle * simPitchRate < 0.0)
    dAngle = -dAngle;

  /***************************************************************/
  /*                                                             */
  /*  arm                                                        */
  /*  gripper                                                    */
  /*                                                             */
  /***************************************************************/

  double position, velocity;
  std_msgs::PR2Arm larm, rarm;
  /* get left arm position */

  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_L_PAN,            &position, &velocity);
  larm.turretAngle       = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_L_SHOULDER_PITCH, &position, &velocity);
  larm.shoulderLiftAngle = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_L_SHOULDER_ROLL,  &position, &velocity);
  larm.upperarmRollAngle = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_L_ELBOW_PITCH,    &position, &velocity);
  larm.elbowAngle        = position; 
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_L_ELBOW_ROLL,     &position, &velocity);
  larm.forearmRollAngle  = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_L_WRIST_PITCH,    &position, &velocity);
  larm.wristPitchAngle   = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_L_WRIST_ROLL,     &position, &velocity);
  larm.wristRollAngle    = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_L_GRIPPER_GAP,    &position, &velocity);
  larm.gripperForceCmd   = velocity;
  larm.gripperGapCmd     = position;
  publish("left_pr2arm_pos", larm);
  /* get right arm position */
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_R_PAN,            &position, &velocity);
  rarm.turretAngle       = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_R_SHOULDER_PITCH, &position, &velocity);
  rarm.shoulderLiftAngle = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_R_SHOULDER_ROLL,  &position, &velocity);
  rarm.upperarmRollAngle = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_R_ELBOW_PITCH,    &position, &velocity);
  rarm.elbowAngle        = position; 
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_R_ELBOW_ROLL,     &position, &velocity);
  rarm.forearmRollAngle  = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_R_WRIST_PITCH,    &position, &velocity);
  rarm.wristPitchAngle   = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_R_WRIST_ROLL,     &position, &velocity);
  rarm.wristRollAngle    = position;
  this->PR2Copy->hw.GetJointPositionActual(PR2::ARM_R_GRIPPER_GAP,    &position, &velocity);
  rarm.gripperForceCmd   = velocity;
  rarm.gripperGapCmd     = position;
  publish("right_pr2arm_pos", rarm);
  
  /***************************************************************/
  /*                                                             */
  /*  frame transforms                                           */
  /*                                                             */
  /*  TODO: should we send z, roll, pitch, yaw? seems to confuse */
  /*        localization                                         */
  /*                                                             */
  /***************************************************************/

  this->PR2Copy->GetBasePositionActual(&x,&y,&z,&roll,&pitch,&yaw); // actual CoM of base

  tf.sendInverseEuler("FRAMEID_ODOM",
               "base",
               x,
               y,
               z - base_center_offset_z, /* get infor from xml: half height of base box */
               yaw,
               pitch,
               roll,
               odomMsg.header.stamp);

  /***************************************************************/
  /*                                                             */
  /*  frame transforms                                           */
  /*                                                             */
  /*  x,y,z,yaw,pitch,roll                                       */
  /*                                                             */
  /***************************************************************/
  tf.sendEuler("base",
               "FRAMEID_ROBOT",
               0,
               0,
               0, 
               0,
               0,
               0,
               odomMsg.header.stamp);

  //std::cout << "base y p r " << yaw << " " << pitch << " " << roll << std::endl;

  // base = center of the bottom of the base box
  // torso = midpoint of bottom of turrets

  tf.sendEuler("torso",
               "base",
               base_torso_offset_x,
               base_torso_offset_y,
               base_torso_offset_z, /* FIXME: spine elevator not accounted for */
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // arm_l_turret = bottom of left turret
  tf.sendEuler("shoulder_pan_left",
               "torso",
               sh_pan_left_torso_offset_x,
               sh_pan_left_torso_offset_y,
               sh_pan_left_torso_offset_z,
               larm.turretAngle,
               //0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);
  //std::cout << "left pan angle " << larm.turretAngle << std::endl;

  // arm_l_shoulder = center of left shoulder pitch bracket
  tf.sendEuler("shoulder_pitch_left",
               "shoulder_pan_left",
               shoulder_pitch_left_offset_x,
               shoulder_pitch_left_offset_y,
               shoulder_pitch_left_offset_z,
               0.0,
               larm.shoulderLiftAngle,
               0.0,
               odomMsg.header.stamp);

  // arm_l_upperarm = upper arm with roll DOF, at shoulder pitch center
  tf.sendEuler("upperarm_roll_left",
               "shoulder_pitch_left",
               upperarm_roll_left_offset_x,
               upperarm_roll_left_offset_y,
               upperarm_roll_left_offset_z,
               0.0,
               0.0,
               larm.upperarmRollAngle,
               odomMsg.header.stamp);

  //frameid_arm_l_elbow = elbow pitch bracket center of rotation
  tf.sendEuler("elbow_flex_left",
               "upperarm_roll_left",
               elbow_flex_left_offset_x,
               elbow_flex_left_offset_y,
               elbow_flex_left_offset_z,
               0.0,
               larm.elbowAngle,
               0.0,
               odomMsg.header.stamp);

  //frameid_arm_l_forearm = forearm roll DOR, at elbow pitch center
  tf.sendEuler("forearm_roll_left",
               "elbow_flex_left",
               forearm_roll_left_offset_x,
               forearm_roll_left_offset_y,
               forearm_roll_left_offset_z,
               0.0,
               0.0,
               larm.forearmRollAngle,
               odomMsg.header.stamp);

  // arm_l_wrist = wrist pitch DOF.
  tf.sendEuler("wrist_flex_left",
               "forearm_roll_left",
               wrist_flex_left_offset_x,
               wrist_flex_left_offset_y,
               wrist_flex_left_offset_z,
               0.0,
               larm.wristPitchAngle,
               0.0,
               odomMsg.header.stamp);

  // arm_l_hand = hand roll DOF, center at wrist pitch center
  tf.sendEuler("gripper_roll_left",
               "wrist_flex_left",
               gripper_roll_left_offset_x,
               gripper_roll_left_offset_y,
               gripper_roll_left_offset_z,
               0.0,
               0.0,
               larm.wristRollAngle,
               odomMsg.header.stamp);

  // proximal digit, left
  tf.sendEuler("finger_l_left",
               "gripper_roll_left",
               finger_l_left_offset_x,
               finger_l_left_offset_y,
               finger_l_left_offset_z,
               0.0,  //FIXME: get angle of finger...
               0.0,
               0.0,
               odomMsg.header.stamp);

  // proximal digit, right
  tf.sendEuler("finger_r_left",
               "gripper_roll_left",
               finger_r_left_offset_x,
               finger_r_left_offset_y,
               finger_r_left_offset_z,
               0.0,  //FIXME: get angle of finger...
               0.0,
               0.0,
               odomMsg.header.stamp);

  // distal digit, left
  tf.sendEuler("finger_tip_l_left",
               "finger_l_left",
               finger_tip_l_left_offset_x,
               finger_tip_l_left_offset_y,
               finger_tip_l_left_offset_z,
               0.0,  //FIXME: get angle of finger tip...
               0.0,
               0.0,
               odomMsg.header.stamp);

  // distal digit, left
  tf.sendEuler("finger_tip_r_left",
               "finger_r_left",
               finger_tip_r_left_offset_x,
               finger_tip_r_left_offset_y,
               finger_tip_r_left_offset_z,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);




  // arm_r_turret = bottom of right turret
  tf.sendEuler("shoulder_pan_right",
               "torso",
               shoulder_pan_right_offset_x,
               shoulder_pan_right_offset_y,
               shoulder_pan_right_offset_z,
               rarm.turretAngle,
               //0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);
  //std::cout << "right pan angle " << larm.turretAngle << std::endl;

  // arm_r_shoulder = center of right shoulder pitch bracket
  tf.sendEuler("shoulder_pitch_right",
               "shoulder_pan_right",
               shoulder_pitch_right_offset_x,
               shoulder_pitch_right_offset_y,
               shoulder_pitch_right_offset_z,
               0.0,
               rarm.shoulderLiftAngle,
               0.0,
               odomMsg.header.stamp);

  // arm_r_upperarm = upper arm with roll DOF, at shoulder pitch center
  tf.sendEuler("upperarm_roll_right",
               "shoulder_pitch_right",
               upperarm_roll_right_offset_x,
               upperarm_roll_right_offset_y,
               upperarm_roll_right_offset_z,
               0.0,
               0.0,
               rarm.upperarmRollAngle,
               odomMsg.header.stamp);

  //frameid_arm_r_elbow = elbow pitch bracket center of rotation
  tf.sendEuler("elbow_flex_right",
               "upperarm_roll_right",
               elbow_flex_right_offset_x,
               elbow_flex_right_offset_y,
               elbow_flex_right_offset_z,
               0.0,
               rarm.elbowAngle,
               0.0,
               odomMsg.header.stamp);

  //frameid_arm_r_forearm = forearm roll DOR, at elbow pitch center
  tf.sendEuler("forearm_roll_right",
               "elbow_flex_right",
               forearm_roll_right_offset_x,
               forearm_roll_right_offset_y,
               forearm_roll_right_offset_z,
               0.0,
               0.0,
               rarm.forearmRollAngle,
               odomMsg.header.stamp);

  // arm_r_wrist = wrist pitch DOF.
  tf.sendEuler("wrist_flex_right",
               "forearm_roll_right",
               wrist_flex_right_offset_x,
               wrist_flex_right_offset_y,
               wrist_flex_right_offset_z,
               0.0,
               rarm.wristPitchAngle,
               0.0,
               odomMsg.header.stamp);

  // arm_r_hand = hand roll DOF, center at wrist pitch center
  tf.sendEuler("gripper_roll_right",
               "wrist_flex_right",
               gripper_roll_right_offset_x,
               gripper_roll_right_offset_y,
               gripper_roll_right_offset_z,
               0.0,
               0.0,
               rarm.wristRollAngle,
               odomMsg.header.stamp);

  // proximal digit, right
  tf.sendEuler("finger_l_right",
               "gripper_roll_right",
               finger_l_right_offset_x,
               finger_l_right_offset_y,
               finger_l_right_offset_z,
               0.0,  //FIXME: get angle of finger...
               0.0,
               0.0,
               odomMsg.header.stamp);

  // proximal digit, right
  tf.sendEuler("finger_r_right",
               "gripper_roll_right",
               finger_r_right_offset_x,
               finger_r_right_offset_y,
               finger_r_right_offset_z,
               0.0,  //FIXME: get angle of finger...
               0.0,
               0.0,
               odomMsg.header.stamp);

  // distal digit, right
  tf.sendEuler("finger_tip_l_right",
               "finger_l_right",
               finger_tip_l_right_offset_x,
               finger_tip_l_right_offset_y,
               finger_tip_l_right_offset_z,
               0.0,  //FIXME: get angle of finger tip...
               0.0,
               0.0,
               odomMsg.header.stamp);

  // distal digit, right
  tf.sendEuler("finger_tip_r_right",
               "finger_r_right",
               finger_tip_r_right_offset_x,
               finger_tip_r_right_offset_y,
               finger_tip_r_right_offset_z,
               0.0,  //FIXME: get angle of finger tip...
               0.0,
               0.0,
               odomMsg.header.stamp);






  // forearm camera left
  tf.sendEuler("forearm_camera_left",
               "forearm_roll_left",
               forearm_camera_left_offset_x,
               forearm_camera_left_offset_y,
               forearm_camera_left_offset_z,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // forearm camera right
  tf.sendEuler("forearm_camera_right",
               "forearm_roll_right",
               forearm_camera_right_offset_x,
               forearm_camera_right_offset_y,
               forearm_camera_right_offset_z,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // wrist camera left
  tf.sendEuler("wrist_camera_left",
               "gripper_roll_left",
               wrist_camera_left_offset_x,
               wrist_camera_left_offset_y,
               wrist_camera_left_offset_z,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // wrist camera right
  tf.sendEuler("wrist_camera_right",
               "gripper_roll_right",
               wrist_camera_right_offset_x,
               wrist_camera_right_offset_y,
               wrist_camera_right_offset_z,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);







  // head pan angle
  tf.sendEuler("head_pan",
               "torso",
               head_pan_offset_x,
               head_pan_offset_y,
               head_pan_offset_z,
               0.0, //FIXME: get pan angle
               0.0,
               0.0,
               odomMsg.header.stamp);

  // head tilt angle
  tf.sendEuler("head_tilt",
               "head_pan",
               head_tilt_offset_x,
               head_tilt_offset_y,
               head_tilt_offset_z,
               0.0, //FIXME: get tilt angle
               0.0,
               0.0,
               odomMsg.header.stamp);

  // FIXME: not implemented
  tf.sendEuler("stereo",
               "head_pan",
               0.0,
               0.0,
               1.10,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // base laser location
  tf.sendEuler("base_laser",
               "base",
               base_laser_offset_x,
               base_laser_offset_y,
               base_laser_offset_z,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // tilt laser location
  double tmpPitch, tmpPitchRate;
  this->PR2Copy->hw.GetJointServoCmd(PR2::HEAD_LASER_PITCH, &tmpPitch, &tmpPitchRate );
  tf.sendEuler("tilt_laser",
               "torso",
               tilt_laser_offset_x,
               tilt_laser_offset_y,
               tilt_laser_offset_z,
               0.0,
               tmpPitch, //FIXME: verify laser tilt angle
               0.0,
               odomMsg.header.stamp);


  /***************************************************************/
  // for the casters
  double tmpSteerFL, tmpVelFL;
  double tmpSteerFR, tmpVelFR;
  double tmpSteerRL, tmpVelRL;
  double tmpSteerRR, tmpVelRR;
  this->PR2Copy->hw.GetJointServoCmd(PR2::CASTER_FL_STEER, &tmpSteerFL, &tmpVelFL );
  this->PR2Copy->hw.GetJointServoCmd(PR2::CASTER_FR_STEER, &tmpSteerFR, &tmpVelFR );
  this->PR2Copy->hw.GetJointServoCmd(PR2::CASTER_RL_STEER, &tmpSteerRL, &tmpVelRL );
  this->PR2Copy->hw.GetJointServoCmd(PR2::CASTER_RR_STEER, &tmpSteerRR, &tmpVelRR );
  tf.sendEuler("caster_front_left",
               "base",
               caster_front_left_offset_x,
               caster_front_left_offset_y,
               caster_front_left_offset_z,
               tmpSteerFL,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("wheel_front_left_l",
               "caster_front_left",
               wheel_front_left_l_offset_x,
               wheel_front_left_l_offset_y,
               wheel_front_left_l_offset_z,
               0.0,
               0.0, //FIXME: get wheel rotation
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("wheel_front_left_r",
               "caster_front_left",
               wheel_front_left_r_offset_x,
               wheel_front_left_r_offset_y,
               wheel_front_left_r_offset_z,
               0.0,
               0.0, //FIXME: get wheel rotation
               0.0,
               odomMsg.header.stamp);

  tf.sendEuler("caster_front_right",
               "base",
               caster_front_right_offset_x,
               caster_front_right_offset_y,
               caster_front_right_offset_z,
               tmpSteerFR,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("wheel_front_right_l",
               "caster_front_right",
               wheel_front_right_l_offset_x,
               wheel_front_right_l_offset_y,
               wheel_front_right_l_offset_z,
               0.0,
               0.0, //FIXME: get wheel rotation
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("wheel_front_right_r",
               "caster_front_right",
               wheel_front_right_r_offset_x,
               wheel_front_right_r_offset_y,
               wheel_front_right_r_offset_z,
               0.0,
               0.0, //FIXME: get wheel rotation
               0.0,
               odomMsg.header.stamp);

  tf.sendEuler("caster_rear_left",
               "base",
               caster_rear_left_offset_x,
               caster_rear_left_offset_y,
               caster_rear_left_offset_z,
               tmpSteerRL,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("wheel_rear_left_l",
               "caster_rear_left",
               wheel_rear_left_l_offset_x,
               wheel_rear_left_l_offset_y,
               wheel_rear_left_l_offset_z,
               0.0,
               0.0, //FIXME: get wheel rotation
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("wheel_rear_left_r",
               "caster_rear_left",
               wheel_rear_left_r_offset_x,
               wheel_rear_left_r_offset_y,
               wheel_rear_left_r_offset_z,
               0.0,
               0.0, //FIXME: get wheel rotation
               0.0,
               odomMsg.header.stamp);

  tf.sendEuler("caster_rear_right",
               "base",
               caster_rear_right_offset_x,
               caster_rear_right_offset_y,
               caster_rear_right_offset_z,
               tmpSteerRR,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("wheel_rear_right_l",
               "caster_rear_right",
               wheel_rear_right_l_offset_x,
               wheel_rear_right_l_offset_y,
               wheel_rear_right_l_offset_z,
               0.0,
               0.0, //FIXME: get wheel rotation
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("wheel_rear_right_r",
               "caster_rear_right",
               wheel_rear_right_r_offset_x,
               wheel_rear_right_r_offset_y,
               wheel_rear_right_r_offset_z,
               0.0,
               0.0, //FIXME: get wheel rotation
               0.0,
               odomMsg.header.stamp);

  publish("transform",this->shutterMsg);
  
  // Publish info on the joints:
  for(RCList::iterator it = RosControllers.begin(); it != RosControllers.end(); ++it)
    (*it)->Update();
  
  for(RJList::iterator it = RosJoints.begin(); it != RosJoints.end(); ++it)
    (*it)->Update();

  this->lock.unlock();
}



