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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// gazebo
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <libpr2API/pr2API.h>
#include "ringbuffer.h"

// roscpp
#include <ros/node.h>
// roscpp - laser
#include <std_msgs/LaserScan.h>
// roscpp - laser image (point cloud)
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/Point3DFloat32.h>
#include <std_msgs/ChannelFloat32.h>
// roscpp - used for shutter message right now
#include <std_msgs/Empty.h>
// roscpp - base
#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/BaseVel.h>
// roscpp - arm
#include <std_msgs/PR2Arm.h>
// roscpp - camera
#include <std_msgs/Image.h>

// for frame transforms
#include <rosTF/rosTF.h>

#include <time.h>
#include <signal.h>

#define MAX_CLOUD_PTS	 100000

// Our node
class GazeboNode : public ros::node
{
  private:
    // Messages that we'll send or receive
    std_msgs::BaseVel velMsg;
    std_msgs::LaserScan laserMsg;
    std_msgs::PointCloudFloat32 cloudMsg;
    std_msgs::Empty shutterMsg;  // marks end of a cloud message
    std_msgs::RobotBase2DOdom odomMsg;

    // A mutex to lock access to fields that are used in message callbacks
    ros::thread::mutex lock;

    // for frame transforms, publish frame transforms
    rosTFServer tf;

    // time step calculation
    double lastTime, simTime;

    // smooth vx, vw commands
    double vxSmooth, vwSmooth;

  public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    GazeboNode(int argc, char** argv, const char* fname);
    ~GazeboNode();

    // advertise / subscribe models
    int SubscribeModels();

    // Do one update of the simulator.  May pause if the next update time
    // has not yet arrived.
    void Update();

    // Message callback for a std_msgs::BaseVel message, which set velocities.
    void cmdvelReceived();

    // Message callback for a std_msgs::PR2Arm message, which sets arm configuration.
    void cmd_leftarmconfigReceived();
    void cmd_rightarmconfigReceived();

    // laser range data
    float    ranges[GZ_LASER_MAX_RANGES];
    uint8_t  intensities[GZ_LASER_MAX_RANGES];

    // camera data
    std_msgs::Image img;
    
    // camera data
    std_msgs::PR2Arm leftarm;
    std_msgs::PR2Arm rightarm;


    // The main simulator object
    PR2::PR2Robot* myPR2;

    // for the point cloud data
    ringBuffer<std_msgs::Point3DFloat32> *cloud_pts;
    ringBuffer<float>             *cloud_ch1;

    static void finalize(int);
};

void
GazeboNode::cmd_rightarmconfigReceived()
{
  this->lock.lock();
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

//	this->myPR2->SetArmJointPosition(PR2::PR2_LEFT_ARM, jointPosition, jointSpeed);
	*/
	//*
	this->myPR2->SetJointServoCmd(PR2::ARM_R_PAN           , this->rightarm.turretAngle,       0);
	this->myPR2->SetJointServoCmd(PR2::ARM_R_SHOULDER_PITCH, this->rightarm.shoulderLiftAngle, 0);
	this->myPR2->SetJointServoCmd(PR2::ARM_R_SHOULDER_ROLL , this->rightarm.upperarmRollAngle, 0);
	this->myPR2->SetJointServoCmd(PR2::ARM_R_ELBOW_PITCH   , this->rightarm.elbowAngle,        0);
	this->myPR2->SetJointServoCmd(PR2::ARM_R_ELBOW_ROLL    , this->rightarm.forearmRollAngle,  0);
	this->myPR2->SetJointServoCmd(PR2::ARM_R_WRIST_PITCH   , this->rightarm.wristPitchAngle,   0);
	this->myPR2->SetJointServoCmd(PR2::ARM_R_WRIST_ROLL    , this->rightarm.wristRollAngle,    0);
	this->myPR2->SetJointServoCmd(PR2::ARM_R_GRIPPER       , this->rightarm.gripperGapCmd,     0);
	this->myPR2->CloseGripper(PR2::PR2_RIGHT_GRIPPER, this->rightarm.gripperGapCmd, this->rightarm.gripperForceCmd);
	//*/
  this->lock.unlock();
}


void
GazeboNode::cmd_leftarmconfigReceived()
{
  this->lock.lock();
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
	this->myPR2->SetArmJointPosition(PR2::PR2_LEFT_ARM, jointPosition, jointSpeed);
	*/

	//*
	this->myPR2->SetJointServoCmd(PR2::ARM_L_PAN           , this->leftarm.turretAngle,       0);
	this->myPR2->SetJointServoCmd(PR2::ARM_L_SHOULDER_PITCH, this->leftarm.shoulderLiftAngle, 0);
	this->myPR2->SetJointServoCmd(PR2::ARM_L_SHOULDER_ROLL , this->leftarm.upperarmRollAngle, 0);
	this->myPR2->SetJointServoCmd(PR2::ARM_L_ELBOW_PITCH   , this->leftarm.elbowAngle,        0);
	this->myPR2->SetJointServoCmd(PR2::ARM_L_ELBOW_ROLL    , this->leftarm.forearmRollAngle,  0);
	this->myPR2->SetJointServoCmd(PR2::ARM_L_WRIST_PITCH   , this->leftarm.wristPitchAngle,   0);
	this->myPR2->SetJointServoCmd(PR2::ARM_L_WRIST_ROLL    , this->leftarm.wristRollAngle,    0);
//	this->myPR2->SetJointServoCmd(PR2::ARM_L_GRIPPER       , this->leftarm.gripperGapCmd,     0);
	this->myPR2->CloseGripper(PR2::PR2_LEFT_GRIPPER, this->leftarm.gripperGapCmd, this->leftarm.gripperForceCmd);
	//*/
  this->lock.unlock();
}

void
GazeboNode::cmdvelReceived()
{
  this->lock.lock();
  double dt;
  double w11, w21, w12, w22;
  w11 = 1.0;
  w21 = 1.0;
  w12 = 1.0;
  w22 = 1.0;

  // smooth out the commands by time decay
  // with w1,w2=1, this means equal weighting for new command every second
  this->myPR2->GetSimTime(&(this->simTime));
  dt = simTime - lastTime;
  //this->vxSmooth = (w11 * this->vxSmooth + w21*dt *this->velMsg.vx)/( w11 + w21*dt);
  //this->vwSmooth = (w12 * this->vwSmooth + w22*dt *this->velMsg.vw)/( w12 + w22*dt);

  this->vxSmooth = this->velMsg.vx;
  this->vwSmooth = this->velMsg.vw;

  fprintf(stderr,"received cmd: %.3f %.3f | %.3f %.3f\n", this->velMsg.vx, this->velMsg.vw,this->vxSmooth, this->vwSmooth);

  this->myPR2->SetBaseSteeringAngle    (this->vxSmooth,0.0,this->vwSmooth);
  while (!this->myPR2->CheckBaseSteeringAngle(M_PI/10.0))
  {
    // do nothing and wait...
    usleep(100000);
  }
  // set base velocity
  this->myPR2->SetJointTorque(PR2::CASTER_FL_DRIVE_L, 1000.0 );
  this->myPR2->SetJointTorque(PR2::CASTER_FR_DRIVE_L, 1000.0 );
  this->myPR2->SetJointTorque(PR2::CASTER_RL_DRIVE_L, 1000.0 );
  this->myPR2->SetJointTorque(PR2::CASTER_RR_DRIVE_L, 1000.0 );
  this->myPR2->SetJointTorque(PR2::CASTER_FL_DRIVE_R, 1000.0 );
  this->myPR2->SetJointTorque(PR2::CASTER_FR_DRIVE_R, 1000.0 );
  this->myPR2->SetJointTorque(PR2::CASTER_RL_DRIVE_R, 1000.0 );
  this->myPR2->SetJointTorque(PR2::CASTER_RR_DRIVE_R, 1000.0 );
  this->myPR2->SetBaseCartesianSpeedCmd(this->vxSmooth, 0.0, this->vwSmooth);

  this->lastTime = this->simTime;

  this->lock.unlock();
}

GazeboNode::GazeboNode(int argc, char** argv, const char* fname) :
        ros::node("rosgazebo"),tf(*this)
{

  // Initialize robot object
  this->myPR2 = new PR2::PR2Robot();
  // Initialize connections
  this->myPR2->InitializeRobot();
  // Set control mode for the base
  this->myPR2->SetBaseControlMode(PR2::PR2_CARTESIAN_CONTROL);

  // Initialize ring buffer for point cloud data
  this->cloud_pts = new ringBuffer<std_msgs::Point3DFloat32>();
  this->cloud_ch1 = new ringBuffer<float>();
  this->cloud_pts->allocate(MAX_CLOUD_PTS);
  this->cloud_ch1->allocate(MAX_CLOUD_PTS);


	/*
	//-------- This doesn't seem to affect anything:
  // Set control mode for the arms
  this->myPR2->SetArmControlMode(PR2::PR2_RIGHT_ARM, PR2::PR2_JOINT_CONTROL);
  this->myPR2->SetArmControlMode(PR2::PR2_LEFT_ARM, PR2::PR2_JOINT_CONTROL);
	//------------------------------------------------------------
	*/
  this->myPR2->EnableGripperLeft();
  this->myPR2->EnableGripperRight();

  this->myPR2->GetSimTime(&(this->lastTime));
  this->myPR2->GetSimTime(&(this->simTime));
}

void GazeboNode::finalize(int)
{
  fprintf(stderr,"Caught sig, clean-up and exit\n");
  sleep(1);
  exit(-1);
}


int
GazeboNode::SubscribeModels()
{
  //advertise<std_msgs::LaserScan>("laser");
  advertise<std_msgs::LaserScan>("scan");
  advertise<std_msgs::RobotBase2DOdom>("odom");
  advertise<std_msgs::Image>("image");
  advertise<std_msgs::PointCloudFloat32>("cloud");
  advertise<std_msgs::Empty>("shutter");
  subscribe("cmd_vel", velMsg, &GazeboNode::cmdvelReceived);
  subscribe("cmd_leftarmconfig", leftarm, &GazeboNode::cmd_leftarmconfigReceived);
  subscribe("cmd_rightarmconfig", rightarm, &GazeboNode::cmd_rightarmconfigReceived);
  return(0);
}

GazeboNode::~GazeboNode()
{
}

void
GazeboNode::Update()
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
  std_msgs::Point3DFloat32 tmp_cloud_pt;

  /***************************************************************/
  /*                                                             */
  /*  laser - pitching                                           */
  /*                                                             */
  /***************************************************************/
  if (this->myPR2->GetLaserRanges(PR2::LASER_HEAD,
                &angle_min, &angle_max, &angle_increment,
                &range_max, &ranges_size     , &ranges_alloc_size,
                &intensities_size, &intensities_alloc_size,
                this->ranges     , this->intensities) == PR2::PR2_ALL_OK)
  {
    for(unsigned int i=0;i<ranges_size;i++)
    {
      // get laser pitch angle
	    double laser_yaw, laser_pitch, laser_pitch_rate;
	    this->myPR2->GetJointServoActual(PR2::HEAD_LASER_PITCH , &laser_pitch,  &laser_pitch_rate);
      // get laser yaw angle
	    laser_yaw = angle_min + (double)i * angle_increment;
	    //std::cout << " pit " << laser_pitch << "yaw " << laser_yaw
	    //          << " amin " <<  angle_min << " inc " << angle_increment << std::endl;
      // populating cloud data by range
      double tmp_range = this->ranges[i];
      // transform from range to x,y,z
      tmp_cloud_pt.x                = tmp_range * cos(laser_yaw) * cos(laser_pitch);
      tmp_cloud_pt.y                = tmp_range * sin(laser_yaw) ; //* cos(laser_pitch);
      tmp_cloud_pt.z                = tmp_range * cos(laser_yaw) * sin(laser_pitch);
      this->cloud_pts->add((std_msgs::Point3DFloat32)tmp_cloud_pt);

      this->cloud_ch1->add(this->intensities[i]);
    }
    /***************************************************************/
    /*                                                             */
    /*  point cloud from laser image                               */
    /*                                                             */
    /***************************************************************/
    //std::cout << " pcd num " << this->cloud_pts->length << std::endl;
    int    num_channels = 1;
    this->cloudMsg.set_pts_size(this->cloud_pts->length);
    this->cloudMsg.set_chan_size(num_channels);
    this->cloudMsg.chan[0].name = "intensities";
    this->cloudMsg.chan[0].set_vals_size(this->cloud_ch1->length);

    for(int i=0;i< this->cloud_pts->length ;i++)
    {
      this->cloudMsg.pts[i].x  = this->cloud_pts->buffer[i].x;
      this->cloudMsg.pts[i].y  = this->cloud_pts->buffer[i].y;
      this->cloudMsg.pts[i].z  = this->cloud_pts->buffer[i].z;
      this->cloudMsg.chan[0].vals[i]    = this->cloud_ch1->buffer[i];
    }
    publish("cloud",this->cloudMsg);
    //publish("shutter",this->shutterMsg);
  }


  this->myPR2->GetSimTime(&(this->simTime));

  /***************************************************************/
  /*                                                             */
  /*  laser - base                                               */
  /*                                                             */
  /***************************************************************/
  if (this->myPR2->GetLaserRanges(PR2::LASER_BASE,
                &angle_min, &angle_max, &angle_increment,
                &range_max, &ranges_size     , &ranges_alloc_size,
                &intensities_size, &intensities_alloc_size,
                this->ranges     , this->intensities) == PR2::PR2_ALL_OK)
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

    this->laserMsg.header.frame_id = FRAMEID_LASER;
    this->laserMsg.header.stamp.sec = (unsigned long)floor(this->simTime);
    this->laserMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->simTime - this->laserMsg.header.stamp.sec) );

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
  this->myPR2->GetBaseCartesianSpeedActual(&vx,&vy,&vw);
  // Translate into ROS message format and publish
  this->odomMsg.vel.x  = vx;
  this->odomMsg.vel.y  = vy;
  this->odomMsg.vel.th = vw;

  // Get position
  double x,y,z,roll,pitch,th;
  this->myPR2->GetBasePositionActual(&x,&y,&z,&roll,&pitch,&th);
  this->odomMsg.pos.x  = x;
  this->odomMsg.pos.y  = y;
  this->odomMsg.pos.th = th;
  // this->odomMsg.stall = this->positionmodel->Stall();

  // TODO: get the frame ID from somewhere
  this->odomMsg.header.frame_id = FRAMEID_ODOM;

  this->odomMsg.header.stamp.sec = (unsigned long)floor(this->simTime);
  this->odomMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->simTime - this->odomMsg.header.stamp.sec) );

  /***************************************************************/
  /*                                                             */
  /*  frame transforms                                           */
  /*                                                             */
  /***************************************************************/
  tf.sendInverseEuler(FRAMEID_ODOM,
                      FRAMEID_ROBOT,
                      odomMsg.pos.x,
                      odomMsg.pos.y,
                      0.0,
                      odomMsg.pos.th,
                      0.0,
                      0.0,
                      odomMsg.header.stamp.sec,
                      odomMsg.header.stamp.nsec);

  // This publish call resets odomMsg.header.stamp.sec and 
  // odomMsg.header.stamp.nsec to zero.  Thus, it must be called *after*
  // those values are reused in the sendInverseEuler() call above.
  publish("odom",this->odomMsg);

  /***************************************************************/
  /*                                                             */
  /*  camera                                                     */
  /*                                                             */
  /***************************************************************/
  uint32_t              width, height, depth;
  std::string           compression, colorspace;
  uint32_t              buf_size;
  static unsigned char  buf[GAZEBO_CAMERA_MAX_IMAGE_SIZE];

  // get image
  this->myPR2->GetCameraImage(PR2::CAMERA_GLOBAL,
          &width           ,         &height               ,
          &depth           ,
          &compression     ,         &colorspace           ,
          &buf_size        ,         buf                   );
  this->img.width       = width;
  this->img.height      = height;
  this->img.compression = compression;
  this->img.colorspace  = colorspace;

  this->img.set_data_size(buf_size);

  this->img.data        = buf;
  //memcpy(this->img.data,buf,data_size);

  publish("image",this->img);

  /***************************************************************/
  /*                                                             */
  /*  pitching Hokuyo joint                                      */
  /*                                                             */
  /***************************************************************/
	static double dAngle = -1;
	double simPitchFreq,simPitchAngle,simPitchRate,simPitchTimeScale,simPitchAmp,simPitchOffset;
	simPitchFreq      = 1.0/10.0;
	simPitchTimeScale = 2.0*M_PI*simPitchFreq;
	simPitchAmp    =  M_PI / 8.0;
	simPitchOffset = -M_PI / 8.0;
	simPitchAngle = simPitchOffset + simPitchAmp * sin(this->simTime * simPitchTimeScale);
	simPitchRate  =  simPitchAmp * simPitchTimeScale * cos(this->simTime * simPitchTimeScale); // TODO: check rate correctness
  this->myPR2->GetSimTime(&this->simTime);
	//std::cout << "sim time: " << this->simTime << std::endl;
	//std::cout << "ang: " << simPitchAngle*180.0/M_PI << "rate: " << simPitchRate*180.0/M_PI << std::endl;
	this->myPR2->SetJointTorque(PR2::HEAD_LASER_PITCH , 1000.0);
  this->myPR2->SetJointGains(PR2::HEAD_LASER_PITCH, 10.0, 0.0, 0.0);
	this->myPR2->SetJointServoCmd(PR2::HEAD_LASER_PITCH , simPitchAngle, simPitchRate);

  if (dAngle * simPitchRate < 0.0)
  {
    dAngle = -dAngle;
    publish("shutter",this->shutterMsg);
  }
	
  // should send shutter when changing direction, or wait for Tully to implement ring buffer in viewer


  /***************************************************************/
  /*                                                             */
  /*  arm                                                        */
  /*  gripper                                                    */
  /*                                                             */
  /***************************************************************/
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
//  this->myPR2->SetGripperGains(PR2::PR2_LEFT_GRIPPER  ,10.0,0.0,0.0);
//  this->myPR2->SetGripperGains(PR2::PR2_RIGHT_GRIPPER ,10.0,0.0,0.0);
//  this->myPR2->OpenGripper(PR2::PR2_LEFT_GRIPPER ,this->arm.gripperGapCmd,this->arm.gripperForceCmd);
//  this->myPR2->CloseGripper(PR2::PR2_RIGHT_GRIPPER,this->arm.gripperGapCmd,this->arm.gripperForceCmd);


  this->lock.unlock();
}



int 
main(int argc, char** argv)
{ 
  ros::init(argc,argv);

  GazeboNode gn(argc,argv,argv[1]);

  signal(SIGINT,  (&gn.finalize));
  signal(SIGQUIT, (&gn.finalize));
  signal(SIGTERM, (&gn.finalize));

  if (gn.SubscribeModels() != 0)
    exit(-1);

  while(1)
  {
    gn.Update();
    usleep(100000);
  }
  
  // have to call this explicitly for some reason.  probably interference
  // from signal handling in Stage / FLTK?
  ros::msg_destruct();

  exit(0);

}
