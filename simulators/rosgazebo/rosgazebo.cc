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



// gazebo
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include "pr2API.hh"

// roscpp
#include <ros/node.h>
// roscpp - laser
#include <std_msgs/MsgLaserScan.h>
// roscpp - base
#include <std_msgs/MsgRobotBase2DOdom.h>
#include <std_msgs/MsgBaseVel.h>
// roscpp - arm
#include <std_msgs/MsgRobotArm.h>
// roscpp - camera
#include <std_msgs/MsgImage.h>

#include <time.h>
#include <signal.h>

#define USAGE "rosgazebo"

// Our node
class GazeboNode : public ros::node
{
  private:
    // Messages that we'll send or receive
    MsgBaseVel velMsg;
    MsgLaserScan laserMsg;
    MsgRobotBase2DOdom odomMsg;

    // A mutex to lock access to fields that are used in message callbacks
    ros::thread::mutex lock;


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

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived();

    // laser range data
    float    ranges[GZ_LASER_MAX_RANGES];
    uint8_t  intensities[GZ_LASER_MAX_RANGES];

    // camera data
    MsgImage img;
    
    // camera data
    MsgRobotArm arm;


    // The main simulator object
    PR2::PR2Robot* myPR2;


    static void finalize(int);
};

void
GazeboNode::cmdvelReceived()
{
  this->lock.lock();

  printf("received cmd: %.3f %.3f\n",
         this->velMsg.vx, this->velMsg.vw);

  // set base velocity
  this->myPR2->SetBaseCartesianSpeedCmd(this->velMsg.vx, 0.0, this->velMsg.vw);
  this->lock.unlock();
}

GazeboNode::GazeboNode(int argc, char** argv, const char* fname) :
        ros::node("rosgazebo")
{

  // Initialize robot object
  this->myPR2 = new PR2::PR2Robot();
  // Initialize connections
  this->myPR2->InitializeRobot();
  // Set control mode for the base
  this->myPR2->SetBaseControlMode(PR2::PR2_CARTESIAN_CONTROL);

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
  advertise<MsgLaserScan>("laser");
  advertise<MsgRobotBase2DOdom>("odom");
  advertise<MsgImage>("image");
  subscribe("cmd_vel", velMsg, &GazeboNode::cmdvelReceived);
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

  /***************************************************************/
  /*                                                             */
  /*  laser                                                      */
  /*                                                             */
  /***************************************************************/
  if (this->myPR2->GetLaserRanges(&angle_min, &angle_max, &angle_increment,
    &range_max, &ranges_size     , &ranges_alloc_size,
                &intensities_size, &intensities_alloc_size,
                this->ranges     , this->intensities) == PR2::PR2_ALL_OK)
  {
    //std::cout << "vx : " << vx << " vy: " << vy << " vw: " << vw << std::endl;

    // Get latest laser data
    // Stg::stg_laser_sample_t* samples = this->lasermodel->GetSamples();
    // if(samples)
    // {
    //   // Translate into ROS message format and publish
    //   Stg::stg_laser_cfg_t cfg = this->lasermodel->GetConfig();
    this->laserMsg.angle_min       = angle_min;
    this->laserMsg.angle_max       = angle_max;
    this->laserMsg.angle_increment = angle_increment;
    this->laserMsg.range_max       = range_max;
    this->laserMsg.set_ranges_size(ranges_size);
    this->laserMsg.set_intensities_size(intensities_size);
    for(unsigned int i=0;i<ranges_size;i++)
    {
      this->laserMsg.ranges[i]      = this->ranges[i];
      this->laserMsg.intensities[i] = this->intensities[i];
    }

    publish("laser",this->laserMsg);
  // }
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
  double x,y,th;
  this->myPR2->GetBasePositionActual(&x,&y,&th);
  this->odomMsg.pos.x  = x;
  this->odomMsg.pos.y  = y;
  this->odomMsg.pos.th = th;
  // this->odomMsg.stall = this->positionmodel->Stall();

  // TODO: get the frame ID from somewhere
  this->odomMsg.header.frame_id = 2;

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
  /*  arm                                                        */
  /*  gripper                                                    */
  /*                                                             */
  /***************************************************************/
  this->arm.turretAngle          = 0.0;
  this->arm.shoulderLiftAngle    = 0.0;
  this->arm.upperarmRollAngle    = 0.0;
  this->arm.elbowAngle           = 0.0;
  this->arm.forearmRollAngle     = 0.0;
  this->arm.wristPitchAngle      = 0.0;
  this->arm.wristRollAngle       = 0.0;
  this->arm.gripperForceCmd      = 1000.0;
  this->arm.gripperGapCmd        = 0.0;

  // gripper test
  this->myPR2->SetGripperGains(PR2::PR2_LEFT_GRIPPER  ,10.0,0.0,0.0);
  this->myPR2->SetGripperGains(PR2::PR2_RIGHT_GRIPPER ,10.0,0.0,0.0);
  this->myPR2->OpenGripper(PR2::PR2_LEFT_GRIPPER ,this->arm.gripperGapCmd,this->arm.gripperForceCmd);
  this->myPR2->CloseGripper(PR2::PR2_RIGHT_GRIPPER,this->arm.gripperGapCmd,this->arm.gripperForceCmd);


  this->lock.unlock();
}



int 
main(int argc, char** argv)
{ 

  // if( argc < 2 )
  // {
  //   puts(USAGE);
  //   exit(-1);
  // }

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
