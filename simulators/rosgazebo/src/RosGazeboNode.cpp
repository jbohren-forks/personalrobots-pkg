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
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_R_GRIPPER       , this->rightarm.gripperGapCmd,     0);
  }

  //Leave a way to communicate with the grippers
  this->PR2Copy->hw.CloseGripper(PR2::PR2_RIGHT_GRIPPER, this->rightarm.gripperGapCmd, this->rightarm.gripperForceCmd);

  //*/
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
    this->PR2Copy->hw.SetJointServoCmd(PR2::ARM_L_GRIPPER       , this->leftarm.gripperGapCmd,     0);
  }

  //Leave a way to communicate with the grippers
  this->PR2Copy->hw.CloseGripper(PR2::PR2_LEFT_GRIPPER, this->leftarm.gripperGapCmd, this->leftarm.gripperForceCmd);
  
  
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

//  KDL::JntArray q = KDL::JntArray(this->PR2Copy->pr2_kin.nJnts);
//  this->PR2Copy->GetArmJointPositionCmd(PR2::PR2_LEFT_ARM, q);
//   this->PR2Copy->SetArmCartesianPosition(PR2::PR2_LEFT_ARM,f);

  this->lock.unlock();
}

bool RosGazeboNode::reset_IK_guess(rosgazebo::VoidVoid::request &req, rosgazebo::VoidVoid::response &res)
{
	this->PR2Copy->GetArmJointPositionCmd(PR2::PR2_RIGHT_ARM, *(this->PR2Copy->pr2_kin.q_IK_guess));
	return true;
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

//  KDL::JntArray q = KDL::JntArray(this->PR2Copy->pr2_kin.nJnts);
//  this->PR2Copy->GetArmJointPositionCmd(PR2::PR2_RIGHT_ARM, q);
//   this->PR2Copy->SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f);

  this->lock.unlock();
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

  // Initialize ring buffer for point cloud data
  this->cloud_pts = new ringBuffer<std_msgs::Point3DFloat32>();
  this->cloud_ch1 = new ringBuffer<float>();
  this->full_cloud_pts = new vector<std_msgs::Point3DFloat32>();
  this->full_cloud_ch1 = new vector<float>();

  // FIXME:  move this to Subscribe Models
  param("tilting_laser/max_cloud_pts",max_cloud_pts, 683); // number of point in one scan line
  this->cloud_pts->allocate(this->max_cloud_pts);
  this->cloud_ch1->allocate(this->max_cloud_pts);
  this->full_cloud_pts->clear();
  this->full_cloud_ch1->clear();

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

  // Initialize ring buffer for point cloud data
  this->cloud_pts = new ringBuffer<std_msgs::Point3DFloat32>();
  this->cloud_ch1 = new ringBuffer<float>();
  this->full_cloud_pts = new vector<std_msgs::Point3DFloat32>();
  this->full_cloud_ch1 = new vector<float>();

  // FIXME:  move this to Subscribe Models
  param("tilting_laser/max_cloud_pts",max_cloud_pts, 683); // number of point in one scan line
  this->cloud_pts->allocate(this->max_cloud_pts);
  this->cloud_ch1->allocate(this->max_cloud_pts);
  this->full_cloud_pts->clear();
  this->full_cloud_ch1->clear();

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
         controller::GripperController      *myGripper,
         controller::JointController** ControllerArray,
         RosJointController ** RosControllerArray):
        ros::node("rosgazebo"),tf(*this),tfc(*this)
{
  // accept passed in robot
  this->PR2Copy = myPR2;

  //Store copy of Controller Array. Only interact with it during Update() calls.
  this->ControllerArray = ControllerArray;
  // initialize random seed
  srand(time(NULL));

  // Initialize ring buffer for point cloud data
  this->cloud_pts = new ringBuffer<std_msgs::Point3DFloat32>();
  this->cloud_ch1 = new ringBuffer<float>();
  this->full_cloud_pts = new vector<std_msgs::Point3DFloat32>();
  this->full_cloud_ch1 = new vector<float>();

  // FIXME:  move this to Subscribe Models
  param("tilting_laser/max_cloud_pts",max_cloud_pts, 683); // number of point in one scan line
  this->cloud_pts->allocate(this->max_cloud_pts);
  this->cloud_ch1->allocate(this->max_cloud_pts);
  this->full_cloud_pts->clear();
  this->full_cloud_ch1->clear();

  // initialize times
  this->PR2Copy->GetTime(&(this->lastTime));
  this->PR2Copy->GetTime(&(this->simTime));

  //No new messages
  newRightArmPos = false;
  newLeftArmPos = false;

  //Use new architecture
  useControllerArray = true;
  
      //Store copy of Controller Array. Only interact with it during Update() calls.
  this->RosControllerArray = RosControllerArray;
}

int
RosGazeboNode::AdvertiseSubscribeMessages()
{
  //advertise<std_msgs::LaserScan>("laser");
  advertise<std_msgs::LaserScan>("scan");
  advertise<std_msgs::RobotBase2DOdom>("odom");
  //advertise<std_msgs::Image>("image");
  advertise<std_msgs::Image>("image");
  advertise<std_msgs::Image>("image_ptz_right");
  advertise<std_msgs::Image>("image_ptz_left");
  advertise<std_msgs::Image>("image_wrist_right");
  advertise<std_msgs::Image>("image_wrist_left");
  advertise<std_msgs::Image>("image_forearm_right");
  advertise<std_msgs::Image>("image_forearm_left");
  advertise<std_msgs::PointCloudFloat32>("cloud");
  advertise<std_msgs::PointCloudFloat32>("full_cloud");
  advertise<std_msgs::PointCloudFloat32>("cloudStereo");
  advertise<std_msgs::Empty>("shutter");
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
  
  for(int i=0; i<PR2::MAX_JOINTS;i++)
    RosControllerArray[i]->advertiseSubscribeMessages();

	//------ services ----------
  advertise_service("reset_IK_guess", &RosGazeboNode::reset_IK_guess);

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

  float    angle_min;
  float    angle_max;
  float    angle_increment;
  float    range_max;
  uint32_t ranges_size;
  uint32_t ranges_alloc_size;
  uint32_t intensities_size;
  uint32_t intensities_alloc_size;
  std_msgs::Point3DFloat32 local_cloud_pt;
  std_msgs::Point3DFloat32 global_cloud_pt;


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
/*  if (this->PR2Copy->hw.GetLaserRanges(PR2::LASER_HEAD,
                &angle_min, &angle_max, &angle_increment,
                &range_max, &ranges_size     , &ranges_alloc_size,
                &intensities_size, &intensities_alloc_size,
                this->ranges     , this->intensities, &tiltLaserTime) == PR2::PR2_ALL_OK)*/if(false)
  {
    for(unsigned int i=0;i<ranges_size;i++)
    {
      // get laser pitch angle
      double laser_yaw, laser_pitch, laser_pitch_rate;
      this->PR2Copy->hw.GetJointServoActual(PR2::HEAD_LASER_PITCH , &laser_pitch,  &laser_pitch_rate);
      // get laser yaw angle
      laser_yaw = angle_min + (double)i * angle_increment;
      //std::cout << " pit " << laser_pitch << "yaw " << laser_yaw
      //          << " amin " <<  angle_min << " inc " << angle_increment << std::endl;
      // populating cloud data by range
      double tmp_range = this->ranges[i];
      // transform from range to x,y,z
      local_cloud_pt.x                = tmp_range * cos(laser_yaw) * cos(laser_pitch);
      local_cloud_pt.y                = tmp_range * sin(laser_yaw) ; //* cos(laser_pitch);
      local_cloud_pt.z                = tmp_range * cos(laser_yaw) * sin(laser_pitch);

      // add gaussian noise
      const double sigma = 0.002;  // 2 millimeter sigma
      local_cloud_pt.x                = local_cloud_pt.x + GaussianKernel(0,sigma);
      local_cloud_pt.y                = local_cloud_pt.y + GaussianKernel(0,sigma);
      local_cloud_pt.z                = local_cloud_pt.z + GaussianKernel(0,sigma);

      // offsets for changing frame of reference to ROBOT FRAME
      // we can use transform server here or use the offsets directly
      local_cloud_pt.x = local_cloud_pt.x + PR2::BASE_TORSO_OFFSET.x + PR2::TORSO_TILT_LASER_OFFSET.x;
      local_cloud_pt.y = local_cloud_pt.y + PR2::BASE_TORSO_OFFSET.y;
      local_cloud_pt.z = local_cloud_pt.z + PR2::BASE_TORSO_OFFSET.z + PR2::TORSO_TILT_LASER_OFFSET.z; /* FIXME: spine elevator not accounted for */

      // get position cheats from simulator
      //double cheat_x,cheat_y,cheat_z,cheat_roll,cheat_pitch,cheat_yaw;
      //this->PR2Copy->GetBasePositionActual(&cheat_x,&cheat_y,&cheat_z,&cheat_roll,&cheat_pitch,&cheat_yaw);
      //global_cloud_pt.x = local_cloud_pt.x + cheat_x;
      //global_cloud_pt.y = local_cloud_pt.y + cheat_y;
      //global_cloud_pt.z = local_cloud_pt.z + cheat_z;
      // apply rotataions yaw, pitch, roll

      // add mixed pixel noise
      // if this point is some threshold away from last, add mixing model

      // push pcd point into structure
      this->cloud_pts->add((std_msgs::Point3DFloat32)local_cloud_pt);
      this->cloud_ch1->add(this->intensities[i]);

      this->full_cloud_pts->push_back((std_msgs::Point3DFloat32)local_cloud_pt);
      this->full_cloud_ch1->push_back(this->intensities[i]);
    }
    /***************************************************************/
    /*                                                             */
    /*  point cloud from laser image                               */
    /*                                                             */
    /***************************************************************/
    //std::cout << " pcd num " << this->cloud_pts->length << std::endl;
    //
    this->cloudMsg.header.frame_id = tf.lookup("FRAMEID_BASE");
    this->cloudMsg.header.stamp.sec = (unsigned long)floor(this->tiltLaserTime);
    this->cloudMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->tiltLaserTime - this->cloudMsg.header.stamp.sec) );

    int    num_channels = 1;
    this->cloudMsg.set_pts_size(this->cloud_pts->length);
    this->cloudMsg.set_chan_size(num_channels);
    this->cloudMsg.chan[0].name = "intensities";
    this->cloudMsg.chan[0].set_vals_size(this->cloud_ch1->length);

    for(int i=0;i< this->cloud_pts->length ;i++)
    {
      this->cloudMsg.pts[i].x        = this->cloud_pts->buffer[i].x;
      this->cloudMsg.pts[i].y        = this->cloud_pts->buffer[i].y;
      this->cloudMsg.pts[i].z        = this->cloud_pts->buffer[i].z;
      this->cloudMsg.chan[0].vals[i] = this->cloud_ch1->buffer[i];
    }

    // TODO: does anyone need transform to MAP FRAME directly here? answer: use 3d world model
    // try
    // {
    //   transformed_cloudMsg = tfc.transformPointCloud("FRAMEID_MAP", this->cloudMsg);
    // }
    // catch(libTF::TransformReference::LookupException& ex)
    // {
    //   puts("no global->local Tx yet");
    //   return;
    // }
    // publish("cloud",transformed_cloudMsg);

    publish("cloud",cloudMsg);
  }


  /***************************************************************/
  /*                                                             */
  /*  laser - base                                               */
  /*                                                             */
  /***************************************************************/
/*  if (this->PR2Copy->hw.GetLaserRanges(PR2::LASER_BASE,
                &angle_min, &angle_max, &angle_increment,
                &range_max, &ranges_size     , &ranges_alloc_size,
                &intensities_size, &intensities_alloc_size,
                this->ranges     , this->intensities, &baseLaserTime) == PR2::PR2_ALL_OK)*/ if(false)
  {
    // Get latest laser data
    this->laserMsg.angle_min       = angle_min;
    this->laserMsg.angle_max       = angle_max;
    this->laserMsg.angle_increment = angle_increment;
    this->laserMsg.range_max       = range_max;
    this->laserMsg.set_ranges_size(ranges_size);
    this->laserMsg.set_intensities_size(intensities_size);
    for(unsigned int i=0;i<ranges_size;i++)
    {
      double tmp_range = this->ranges[i];
      this->laserMsg.ranges[i]      =tmp_range;
      this->laserMsg.intensities[i] = this->intensities[i];
    }

    this->laserMsg.header.frame_id = tf.lookup("FRAMEID_LASER");
    this->laserMsg.header.stamp.sec = (unsigned long)floor(this->baseLaserTime);
    this->laserMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->baseLaserTime - this->laserMsg.header.stamp.sec) );

    //publish("laser",this->laserMsg); // for laser_view FIXME: can alias this at the commandline or launch script
    publish("scan",this->laserMsg);  // for rosstage
  }



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

  /***************************************************************/
  /*                                                             */
  /*  frame transforms                                           */
  /*                                                             */
  /*  TODO: should we send z, roll, pitch, yaw? seems to confuse */
  /*        localization                                         */
  /*                                                             */
  /***************************************************************/
  tf.sendInverseEuler("FRAMEID_ODOM",
                      "FRAMEID_ROBOT",
                      odomMsg.pos.x,
                      odomMsg.pos.y,
                      0.0,
                      odomMsg.pos.th,
                      0.0,
                      0.0,
                      odomMsg.header.stamp);

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
  uint32_t              width, height, depth;
  std::string           compression, colorspace;
  uint32_t              buf_size;
  static unsigned char  buf_ptz_right[GAZEBO_CAMERA_MAX_IMAGE_SIZE];
  static unsigned char  buf_ptz_left[GAZEBO_CAMERA_MAX_IMAGE_SIZE];
  static unsigned char  buf_wrist_right[GAZEBO_CAMERA_MAX_IMAGE_SIZE];
  static unsigned char  buf_wrist_left[GAZEBO_CAMERA_MAX_IMAGE_SIZE];
  static unsigned char  buf_forearm_right[GAZEBO_CAMERA_MAX_IMAGE_SIZE];
  static unsigned char  buf_forearm_left[GAZEBO_CAMERA_MAX_IMAGE_SIZE];

  //this->PR2Copy->hw.GetCameraImage(PR2::CAMERA_GLOBAL,
  // ----------------------- get image ----------------------------
  /*if (PR2::PR2_ALL_OK == this->PR2Copy->hw.GetCameraImage(PR2::CAMERA_HEAD_RIGHT,
          &width           ,         &height               ,
          &depth           ,
          &compression     ,         &colorspace           ,
          &buf_size        ,         buf_ptz_right         , &cameraTime)) */if(false){
    this->img_ptz_right.width       = width;
    this->img_ptz_right.height      = height;
    this->img_ptz_right.compression = compression;
    this->img_ptz_right.colorspace  = colorspace;

    if(buf_size >0)
    {
      this->img_ptz_right.header.frame_id = tf.lookup("FRAMEID_PTZ_CAM_R");
      this->img_ptz_right.header.stamp.sec = (unsigned long)floor(this->cameraTime);
      this->img_ptz_right.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->cameraTime - this->laserMsg.header.stamp.sec) );

      this->img_ptz_right.set_data_size(buf_size);

      this->img_ptz_right.data        = buf_ptz_right;
      //memcpy(this->img_ptz_right.data,buf,data_size);

      //publish("image",this->img_ptz_right);
      publish("image_ptz_right",this->img_ptz_right);
    }
  }
  // ----------------------- get image ----------------------------
  /*if (PR2::PR2_ALL_OK == this->PR2Copy->hw.GetCameraImage(PR2::CAMERA_HEAD_LEFT,
          &width           ,         &height               ,
          &depth           ,
          &compression     ,         &colorspace           ,
          &buf_size        ,         buf_ptz_left          , &cameraTime))*/if(false) {
    this->img_ptz_left .width       = width;
    this->img_ptz_left .height      = height;
    this->img_ptz_left .compression = compression;
    this->img_ptz_left .colorspace  = colorspace;

    if(buf_size >0)
    {
      this->img_ptz_left.header.frame_id = tf.lookup("FRAMEID_PTZ_CAM_L");
      this->img_ptz_left.header.stamp.sec = (unsigned long)floor(this->cameraTime);
      this->img_ptz_left.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->cameraTime - this->laserMsg.header.stamp.sec) );

      this->img_ptz_left .set_data_size(buf_size);

      this->img_ptz_left .data        = buf_ptz_left ;
      //memcpy(this->img_ptz_left .data,buf,data_size);

      publish("image_ptz_left",this->img_ptz_left);
    }
  }

  // ----------------------- get image ----------------------------
  if (PR2::PR2_ALL_OK == this->PR2Copy->hw.GetCameraImage(PR2::CAMERA_WRIST_RIGHT,
          &width           ,         &height               ,
          &depth           ,
          &compression     ,         &colorspace           ,
          &buf_size        ,         buf_wrist_right       , &cameraTime  )) {
    this->img_wrist_right.width       = width;
    this->img_wrist_right.height      = height;
    this->img_wrist_right.compression = compression;
    this->img_wrist_right.colorspace  = colorspace;

    if(buf_size >0)
    {
      this->img_wrist_right.header.frame_id = tf.lookup("FRAMEID_WRIST_CAM_R");
      this->img_wrist_right.header.stamp.sec = (unsigned long)floor(this->cameraTime);
      this->img_wrist_right.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->cameraTime - this->laserMsg.header.stamp.sec) );

      this->img_wrist_right.set_data_size(buf_size);

      this->img_wrist_right.data        = buf_wrist_right;
      //memcpy(this->img_wrist_right.data,buf,data_size);

      publish("image_wrist_right",this->img_wrist_right);
    }
  }

  // ----------------------- get image ----------------------------
  if (PR2::PR2_ALL_OK == this->PR2Copy->hw.GetCameraImage(PR2::CAMERA_WRIST_LEFT,
          &width           ,         &height               ,
          &depth           ,
          &compression     ,         &colorspace           ,
          &buf_size        ,         buf_wrist_left        , &cameraTime  )) {
    this->img_wrist_left .width       = width;
    this->img_wrist_left .height      = height;
    this->img_wrist_left .compression = compression;
    this->img_wrist_left .colorspace  = colorspace;

    if(buf_size >0)
    {
      this->img_wrist_left.header.frame_id = tf.lookup("FRAMEID_WRIST_CAM_L");
      this->img_wrist_left.header.stamp.sec = (unsigned long)floor(this->cameraTime);
      this->img_wrist_left.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->cameraTime - this->laserMsg.header.stamp.sec) );

      this->img_wrist_left .set_data_size(buf_size);

      this->img_wrist_left .data        = buf_wrist_left ;
      //memcpy(this->img_wrist_left .data,buf,data_size);

      publish("image_wrist_left",this->img_wrist_left);
    }
  }

  // ----------------------- get image ----------------------------
  if (PR2::PR2_ALL_OK == this->PR2Copy->hw.GetCameraImage(PR2::CAMERA_FOREARM_RIGHT,
          &width           ,         &height               ,
          &depth           ,
          &compression     ,         &colorspace           ,
          &buf_size        ,         buf_forearm_right     , &cameraTime    )){
    this->img_forearm_right.width       = width;
    this->img_forearm_right.height      = height;
    this->img_forearm_right.compression = compression;
    this->img_forearm_right.colorspace  = colorspace;

    if(buf_size >0)
    {
      this->img_forearm_right.header.frame_id = tf.lookup("FRAMEID_FOREARM_CAM_R");
      this->img_forearm_right.header.stamp.sec = (unsigned long)floor(this->cameraTime);
      this->img_forearm_right.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->cameraTime - this->laserMsg.header.stamp.sec) );

      this->img_forearm_right.set_data_size(buf_size);

      this->img_forearm_right.data        = buf_forearm_right;
      //memcpy(this->img_forearm_right.data,buf,data_size);

      publish("image_forearm_right",this->img_forearm_right);
    }
  }

  // ----------------------- get image ----------------------------
  if (PR2::PR2_ALL_OK == this->PR2Copy->hw.GetCameraImage(PR2::CAMERA_FOREARM_LEFT,
          &width           ,         &height               ,
          &depth           ,
          &compression     ,         &colorspace           ,
          &buf_size        ,         buf_forearm_left      , &cameraTime    )) {
    this->img_forearm_left .width       = width;
    this->img_forearm_left .height      = height;
    this->img_forearm_left .compression = compression;
    this->img_forearm_left .colorspace  = colorspace;

    if(buf_size >0)
    {
      this->img_forearm_left.header.frame_id = tf.lookup("FRAMEID_FOREARM_CAM_L");
      this->img_forearm_left.header.stamp.sec = (unsigned long)floor(this->cameraTime);
      this->img_forearm_left.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->cameraTime - this->laserMsg.header.stamp.sec) );

      this->img_forearm_left .set_data_size(buf_size);

      this->img_forearm_left .data        = buf_forearm_left ;
      //memcpy(this->img_forearm_left .data,buf,data_size);

      publish("image_forearm_left",this->img_forearm_left);
    }
  }

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
  //std::cout << "sim time: " << this->simTime << std::endl;
  //std::cout << "ang: " << simPitchAngle*180.0/M_PI << "rate: " << simPitchRate*180.0/M_PI << std::endl;
  this->PR2Copy->hw.SetJointTorque(PR2::HEAD_LASER_PITCH , 200.0);
  this->PR2Copy->hw.SetJointGains(PR2::HEAD_LASER_PITCH, 3.0, 1.0, 0.0);
  this->PR2Copy->hw.SetJointServoCmd(PR2::HEAD_LASER_PITCH , simPitchAngle, simPitchRate);

  if (dAngle * simPitchRate < 0.0)
  {
    // shutter in irrlicht viewer clears the cloud memory, goes before republish of full_cloud
    publish("shutter",this->shutterMsg);

    dAngle = -dAngle;

    this->full_cloudMsg.header.frame_id = tf.lookup("FRAMEID_BASE");
    this->full_cloudMsg.header.stamp.sec = (unsigned long)floor(this->tiltLaserTime);
    this->full_cloudMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->tiltLaserTime - this->full_cloudMsg.header.stamp.sec) );

    int    num_channels = 1;
    this->full_cloudMsg.set_pts_size(this->full_cloud_pts->size());
    this->full_cloudMsg.set_chan_size(num_channels);
    this->full_cloudMsg.chan[0].name = "intensities";
    this->full_cloudMsg.chan[0].set_vals_size(this->full_cloud_ch1->size());
    // TODO: make sure this is doing the right memcopy stuff
    //memcpy(this->full_cloudMsg.pts          , &(this->full_cloud_pts->front()), this->full_cloud_pts->size());
    //memcpy(this->full_cloudMsg.chan[0].vals , &(this->full_cloud_ch1->front()), this->full_cloud_ch1->size());

    for(unsigned int i=0;i< this->full_cloud_pts->size() ;i++)
    {
      this->full_cloudMsg.pts[i].x        = (this->full_cloud_pts->at(i)).x;
      this->full_cloudMsg.pts[i].y        = (this->full_cloud_pts->at(i)).y;
      this->full_cloudMsg.pts[i].z        = (this->full_cloud_pts->at(i)).z;
      this->full_cloudMsg.chan[0].vals[i] = (this->full_cloud_ch1->at(i));
    }

    publish("full_cloud",this->full_cloudMsg);
    this->full_cloud_pts->clear();
    this->full_cloud_ch1->clear();

  }

  // should send shutter when changing direction, or wait for Tully to implement ring buffer in viewer


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
  this->PR2Copy->GetLeftGripperActual  (                              &position, &velocity);
  larm.gripperForceCmd   = velocity;
  larm.gripperGapCmd     = position;
  publish("left_pr2arm_pos", larm);
  /* get left arm position */
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
  this->PR2Copy->GetRightGripperActual  (                             &position, &velocity);
  rarm.gripperForceCmd   = velocity;
  rarm.gripperGapCmd     = position;
  publish("right_pr2arm_pos", rarm);
  

  //  this->arm.turretAngle          = 0.0;
  //  this->arm.shoulderLiftAngle    = 0.0;
  //  this->arm.upperarmRollAngle    = 0.0;
  //  this->arm.elbowAngle           = 0.0;
  //  this->arm.forearmRollAngle     = 0.0;
  //  this->arm.wristPitchAngle      = 0.0;
  //  this->arm.wristRollAngle       = 0.0;
  //  this->arm.gripperForceCmd      = 1000.0;
  //  this->arm.gripperGapCmd        = 0.0;
  //
  //  // gripper test
  //  this->PR2Copy->SetGripperGains(PR2::PR2_LEFT_GRIPPER  ,10.0,0.0,0.0);
  //  this->PR2Copy->SetGripperGains(PR2::PR2_RIGHT_GRIPPER ,10.0,0.0,0.0);
  //  this->PR2Copy->OpenGripper(PR2::PR2_LEFT_GRIPPER ,this->arm.gripperGapCmd,this->arm.gripperForceCmd);
  //  this->PR2Copy->CloseGripper(PR2::PR2_RIGHT_GRIPPER,this->arm.gripperGapCmd,this->arm.gripperForceCmd);

  /***************************************************************/
  /*                                                             */
  /*  frame transforms                                           */
  /*                                                             */
  /*  x,y,z,yaw,pitch,roll                                       */
  /*                                                             */
  /***************************************************************/
  //this->PR2Copy->GetBasePositionActual(&x,&y,&z,&roll,&pitch,&yaw); // actual CoM of base
  tf.sendEuler("FRAMEID_BASE",
               "FRAMEID_ODOM",
               x,
               y,
               z-0.13, /* half height of base box */
               yaw,
               pitch,
               roll,
               odomMsg.header.stamp);
  //std::cout << "base y p r " << yaw << " " << pitch << " " << roll << std::endl;

  // base = center of the bottom of the base box
  // torso = midpoint of bottom of turrets
  tf.sendEuler("FRAMEID_TORSO",
               "FRAMEID_BASE",
               PR2::BASE_TORSO_OFFSET.x,
               PR2::BASE_TORSO_OFFSET.y,
               PR2::BASE_TORSO_OFFSET.z, /* FIXME: spine elevator not accounted for */
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // arm_l_turret = bottom of left turret
  tf.sendEuler("FRAMEID_ARM_L_TURRET",
               "FRAMEID_TORSO",
               PR2::TORSO_LEFT_ARM_PAN_OFFSET.x,
               PR2::TORSO_LEFT_ARM_PAN_OFFSET.y,
               PR2::TORSO_LEFT_ARM_PAN_OFFSET.z,
               larm.turretAngle,
               //0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);
  //std::cout << "left pan angle " << larm.turretAngle << std::endl;

  // arm_l_shoulder = center of left shoulder pitch bracket
  tf.sendEuler("FRAMEID_ARM_L_SHOULDER",
               "FRAMEID_ARM_L_TURRET",
               PR2::ARM_PAN_SHOULDER_PITCH_OFFSET.x,
               PR2::ARM_PAN_SHOULDER_PITCH_OFFSET.y,
               PR2::ARM_PAN_SHOULDER_PITCH_OFFSET.z,
               0.0,
               larm.shoulderLiftAngle,
               0.0,
               odomMsg.header.stamp);

  // arm_l_upperarm = upper arm with roll DOF, at shoulder pitch center
  tf.sendEuler("FRAMEID_ARM_L_UPPERARM",
               "FRAMEID_ARM_L_SHOULDER",
               PR2::ARM_SHOULDER_PITCH_ROLL_OFFSET.x,
               PR2::ARM_SHOULDER_PITCH_ROLL_OFFSET.y,
               PR2::ARM_SHOULDER_PITCH_ROLL_OFFSET.z,
               0.0,
               0.0,
               larm.upperarmRollAngle,
               odomMsg.header.stamp);

  //frameid_arm_l_elbow = elbow pitch bracket center of rotation
  tf.sendEuler("FRAMEID_ARM_L_ELBOW",
               "FRAMEID_ARM_L_UPPERARM",
               PR2::ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET.x,
               PR2::ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET.y,
               PR2::ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET.z,
               0.0,
               larm.elbowAngle,
               0.0,
               odomMsg.header.stamp);

  //frameid_arm_l_forearm = forearm roll DOR, at elbow pitch center
  tf.sendEuler("FRAMEID_ARM_L_FOREARM",
               "FRAMEID_ARM_L_ELBOW",
               PR2::ELBOW_PITCH_ELBOW_ROLL_OFFSET.x,
               PR2::ELBOW_PITCH_ELBOW_ROLL_OFFSET.y,
               PR2::ELBOW_PITCH_ELBOW_ROLL_OFFSET.z,
               0.0,
               0.0,
               larm.forearmRollAngle,
               odomMsg.header.stamp);

  // arm_l_wrist = wrist pitch DOF.
  tf.sendEuler("FRAMEID_ARM_L_WRIST",
               "FRAMEID_ARM_L_FOREARM",
               PR2::ELBOW_ROLL_WRIST_PITCH_OFFSET.x,
               PR2::ELBOW_ROLL_WRIST_PITCH_OFFSET.y,
               PR2::ELBOW_ROLL_WRIST_PITCH_OFFSET.z,
               0.0,
               larm.wristPitchAngle,
               0.0,
               odomMsg.header.stamp);

  // arm_l_hand = hand roll DOF, center at wrist pitch center
  tf.sendEuler("FRAMEID_ARM_L_HAND",
               "FRAMEID_ARM_L_WRIST",
               PR2::WRIST_PITCH_WRIST_ROLL_OFFSET.x,
               PR2::WRIST_PITCH_WRIST_ROLL_OFFSET.y,
               PR2::WRIST_PITCH_WRIST_ROLL_OFFSET.z,
               0.0,
               0.0,
               larm.wristRollAngle,
               odomMsg.header.stamp);

  // FIXME: not implemented
  tf.sendEuler("FRAMEID_ARM_L_FINGER_1",
               "FRAMEID_ARM_L_HAND",
               0.05,
               0.025,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // FIXME: not implemented
  tf.sendEuler("FRAMEID_ARM_L_FINGER_2",
               "FRAMEID_ARM_L_HAND",
               0.05,
               -0.025,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);


  // arm_r_turret = bottom of right turret
  tf.sendEuler("FRAMEID_ARM_R_TURRET",
               "FRAMEID_TORSO",
               PR2::TORSO_RIGHT_ARM_PAN_OFFSET.x,
               PR2::TORSO_RIGHT_ARM_PAN_OFFSET.y,
               PR2::TORSO_RIGHT_ARM_PAN_OFFSET.z,
               rarm.turretAngle,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // arm_r_shoulder = center of right shoulder pitch bracket
  tf.sendEuler("FRAMEID_ARM_R_SHOULDER",
               "FRAMEID_ARM_R_TURRET",
               PR2::ARM_PAN_SHOULDER_PITCH_OFFSET.x,
               PR2::ARM_PAN_SHOULDER_PITCH_OFFSET.y,
               PR2::ARM_PAN_SHOULDER_PITCH_OFFSET.z,
               0.0,
               rarm.shoulderLiftAngle,
               0.0,
               odomMsg.header.stamp);

  // arm_r_upperarm = upper arm with roll DOF, at shoulder pitch center
  tf.sendEuler("FRAMEID_ARM_R_UPPERARM",
               "FRAMEID_ARM_R_SHOULDER",
               PR2::ARM_SHOULDER_PITCH_ROLL_OFFSET.x,
               PR2::ARM_SHOULDER_PITCH_ROLL_OFFSET.y,
               PR2::ARM_SHOULDER_PITCH_ROLL_OFFSET.z,
               0.0,
               0.0,
               rarm.upperarmRollAngle,
               odomMsg.header.stamp);

  //frameid_arm_r_elbow = elbow pitch bracket center of rotation
  tf.sendEuler("FRAMEID_ARM_R_ELBOW",
               "FRAMEID_ARM_R_UPPERARM",
               PR2::ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET.x,
               PR2::ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET.y,
               PR2::ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET.z,
               0.0,
               rarm.elbowAngle,
               0.0,
               odomMsg.header.stamp);

  //frameid_arm_r_forearm = forearm roll DOR, at elbow pitch center
  tf.sendEuler("FRAMEID_ARM_R_FOREARM",
               "FRAMEID_ARM_R_ELBOW",
               PR2::ELBOW_PITCH_ELBOW_ROLL_OFFSET.x,
               PR2::ELBOW_PITCH_ELBOW_ROLL_OFFSET.y,
               PR2::ELBOW_PITCH_ELBOW_ROLL_OFFSET.z,
               0.0,
               0.0,
               rarm.forearmRollAngle,
               odomMsg.header.stamp);

  // arm_r_wrist = wrist pitch DOF.
  tf.sendEuler("FRAMEID_ARM_R_WRIST",
               "FRAMEID_ARM_R_FOREARM",
               PR2::ELBOW_ROLL_WRIST_PITCH_OFFSET.x,
               PR2::ELBOW_ROLL_WRIST_PITCH_OFFSET.y,
               PR2::ELBOW_ROLL_WRIST_PITCH_OFFSET.z,
               0.0,
               rarm.wristPitchAngle,
               0.0,
               odomMsg.header.stamp);

  // arm_r_hand = hand roll DOF, center at wrist pitch center
  tf.sendEuler("FRAMEID_ARM_R_HAND",
               "FRAMEID_ARM_R_WRIST",
               PR2::WRIST_PITCH_WRIST_ROLL_OFFSET.x,
               PR2::WRIST_PITCH_WRIST_ROLL_OFFSET.y,
               PR2::WRIST_PITCH_WRIST_ROLL_OFFSET.z,
               0.0,
               0.0,
               rarm.wristRollAngle,
               odomMsg.header.stamp);

  // FIXME: not implemented
  tf.sendEuler("FRAMEID_ARM_R_FINGER_1",
               "FRAMEID_ARM_R_HAND",
               0.05,
               0.025,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // FIXME: not implemented
  tf.sendEuler("FRAMEID_ARM_R_FINGER_2",
               "FRAMEID_ARM_R_HAND",
               0.05,
               -0.025,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // FIXME: not implemented
  tf.sendEuler("FRAMEID_HEAD_PAN_BASE",
               "FRAMEID_TORSO",
               PR2::TORSO_HEAD_OFFSET.x,
               0.0,
               PR2::TORSO_HEAD_OFFSET.z,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // FIXME: not implemented
  tf.sendEuler("FRAMEID_HEAD_TILT_BASE",
               "FRAMEID_HEAD_PAN_BASE",
               PR2::HEAD_PAN_HEAD_PITCH_OFFSET.x,
               0.0,
               PR2::HEAD_PAN_HEAD_PITCH_OFFSET.z,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // FIXME: not implemented
  tf.sendEuler("FRAMEID_STEREO_BLOCK",
               "FRAMEID_TORSO",
               0.0,
               0.0,
               1.10,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // FIXME: not implemented
  tf.sendEuler("FRAMEID_TILT_LASER_BLOCK",
               "FRAMEID_TORSO",
               PR2::TORSO_TILT_LASER_OFFSET.x,
               0.0,
               PR2::TORSO_TILT_LASER_OFFSET.z,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  // FIXME: not implemented
  tf.sendEuler("FRAMEID_BASE_LASER_BLOCK",
               "FRAMEID_BASE",
               PR2::BASE_BASE_LASER_OFFSET.x,
               0.0,
               PR2::BASE_BASE_LASER_OFFSET.z,
               0.0,
               0.0,
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
  tf.sendEuler("FRAMEID_CASTER_FL_BODY",
               "FRAMEID_BASE",
               PR2::BASE_BODY_OFFSETS[0].x,
               PR2::BASE_BODY_OFFSETS[0].y,
               PR2::BASE_BODY_OFFSETS[0].z,
               tmpSteerFL,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("FRAMEID_CASTER_FL_WHEEL_L",
               "FRAMEID_CASTER_FL_BODY",
               0.0,
               PR2::CASTER_DRIVE_OFFSET[0].y,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("FRAMEID_CASTER_FL_WHEEL_R",
               "FRAMEID_CASTER_FL_BODY",
               0.0,
               PR2::CASTER_DRIVE_OFFSET[1].y,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  tf.sendEuler("FRAMEID_CASTER_FR_BODY",
               "FRAMEID_BASE",
               PR2::BASE_BODY_OFFSETS[3].x,
               PR2::BASE_BODY_OFFSETS[3].y,
               PR2::BASE_BODY_OFFSETS[3].z,
               tmpSteerFR,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("FRAMEID_CASTER_FR_WHEEL_L",
               "FRAMEID_CASTER_FR_BODY",
               0.0,
               PR2::CASTER_DRIVE_OFFSET[2].y,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("FRAMEID_CASTER_FR_WHEEL_R",
               "FRAMEID_CASTER_FR_BODY",
               0.0,
               PR2::CASTER_DRIVE_OFFSET[3].y,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  tf.sendEuler("FRAMEID_CASTER_RL_BODY",
               "FRAMEID_BASE",
               PR2::BASE_BODY_OFFSETS[6].x,
               PR2::BASE_BODY_OFFSETS[6].y,
               PR2::BASE_BODY_OFFSETS[6].z,
               tmpSteerRL,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("FRAMEID_CASTER_RL_WHEEL_L",
               "FRAMEID_CASTER_RL_BODY",
               0.0,
               PR2::CASTER_DRIVE_OFFSET[4].y,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("FRAMEID_CASTER_RL_WHEEL_R",
               "FRAMEID_CASTER_RL_BODY",
               0.0,
               PR2::CASTER_DRIVE_OFFSET[5].y,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);

  tf.sendEuler("FRAMEID_CASTER_RR_BODY",
               "FRAMEID_BASE",
               PR2::BASE_BODY_OFFSETS[9].x,
               PR2::BASE_BODY_OFFSETS[9].y,
               PR2::BASE_BODY_OFFSETS[9].z,
               tmpSteerRR,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("FRAMEID_CASTER_RR_WHEEL_L",
               "FRAMEID_CASTER_RR_BODY",
               0.0,
               PR2::CASTER_DRIVE_OFFSET[6].y,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);
  tf.sendEuler("FRAMEID_CASTER_RR_WHEEL_R",
               "FRAMEID_CASTER_RR_BODY",
               0.0,
               PR2::CASTER_DRIVE_OFFSET[7].y,
               0.0,
               0.0,
               0.0,
               0.0,
               odomMsg.header.stamp);


	publish("transform",this->shutterMsg);
  
  // Publish info on the joints:
  for(int i=0; i<PR2::MAX_JOINTS; i++)
    RosControllerArray[i]->update();
  
  this->lock.unlock();
}



