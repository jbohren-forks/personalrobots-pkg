/*
 *  rosstage
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

/**

@mainpage

@htmlinclude manifest.html

@b rosstage wraps the Stage 2-D multi-robot simulator, via @b libstage.

For detailed documentation,
consult the <a href="http://playerstage.sourceforge.net/doc/stage-cvs">Stage manual</a>.

This node finds the first Stage model of type @b laser, and the first model
of type @b position, and maps these models to the ROS topics given below.
If a laser and a position model are not found, rosstage exits.

@todo Define a more general method for mapping Stage models onto ROS topics
/ services.  Something like the Player/Stage model, in which a Player .cfg
file is used to map named Stage models onto Player devices, is probably the
way to go.  The same technique can be used for rosgazebo.

<hr>

@section usage Usage
@verbatim
$ rosstage <world> [standard ROS args]
@endverbatim

@param world The Stage .world file to load.

@par Example

@verbatim
$ rosstage willow-erratic.world
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "cmd_vel"/BaseVel : velocity commands to differentially drive the 
position model.

Publishes to (name / type):
- @b "odom"/RobotBase2DOdom : odometry data from the position model.
- @b "base_scan"/LaserScan : scans from the laser model.


<hr>

@section parameters ROS parameters

- None


 **/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// libstage
#include <stage.hh>

// roscpp
#include <ros/node.h>
#include <std_msgs/LaserScan.h>
#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/TransformWithRateStamped.h>
#include <std_msgs/Pose3D.h>
#include <std_msgs/BaseVel.h>

#include "rosTF/rosTF.h"

#define USAGE "rosstage <worldfile>"

// Our node
class StageNode : public ros::node
{
  private:
    // Messages that we'll send or receive
    std_msgs::BaseVel velMsg;
    std_msgs::LaserScan laserMsg;
    std_msgs::RobotBase2DOdom odomMsg;
    std_msgs::TransformWithRateStamped groundTruthMsg;

    // A mutex to lock access to fields that are used in message callbacks
    ros::thread::mutex lock;

    // The models that we're interested in
    Stg::StgModelLaser* lasermodel;
    Stg::StgModelPosition* positionmodel;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static void ghfunc(gpointer key, Stg::StgModel* mod, StageNode* node);

  rosTFServer tf;

  public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    StageNode(int argc, char** argv, const char* fname);
    ~StageNode();

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Do one update of the simulator.  May pause if the next update time
    // has not yet arrived.
    void Update();

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived();

    // The main simulator object
    Stg::StgWorldGui* world;
};

void
StageNode::ghfunc(gpointer key,
                  Stg::StgModel* mod,
                  StageNode* node)
{
  if(!(node->lasermodel) &&
     (node->lasermodel = dynamic_cast<Stg::StgModelLaser*>(mod)))
  {
    puts("found laser");
  }
  if(!(node->positionmodel) &&
     (node->positionmodel = dynamic_cast<Stg::StgModelPosition*>(mod)))
  {
    puts("found position");
  }
}

void
StageNode::cmdvelReceived()
{
  this->lock.lock();

  this->positionmodel->SetSpeed(this->velMsg.vx, 0.0, this->velMsg.vw);
  this->lock.unlock();
}

StageNode::StageNode(int argc, char** argv, const char* fname) :
  ros::node("rosstage"),
  tf(*this)
{
  this->lasermodel = NULL;
  this->positionmodel = NULL;

  // initialize libstage
  Stg::Init( &argc, &argv );
  //StgWorld world;
  this->world = new Stg::StgWorldGui(800, 700, "Stage (ROS)");

  this->world->Load(fname);

  this->world->ForEachModel((GHFunc)ghfunc, this);
}

// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int
StageNode::SubscribeModels()
{
  if(this->lasermodel)
    this->lasermodel->Subscribe();
  else
  {
    puts("no laser");
    return(-1);
  }
  if(this->positionmodel)
    this->positionmodel->Subscribe();
  else
  {
    puts("no position");
    return(-1);
  }

  advertise<std_msgs::LaserScan>("base_scan",10);
  advertise<std_msgs::RobotBase2DOdom>("odom",10);
  advertise<std_msgs::TransformWithRateStamped>("base_pose_ground_truth",10);
  subscribe("cmd_vel", velMsg, &StageNode::cmdvelReceived, 10);
  return(0);
}

StageNode::~StageNode()
{
}

void
StageNode::Update()
{
  // Wait until it's time to update
  this->world->PauseUntilNextUpdateTime();
  this->lock.lock();

  // Let the simulator update (it will sleep if there's time)
  this->world->Update();

  ros::Time sim_time;
  sim_time.fromSec(world->SimTimeNow() / 1e6);

  // Get latest laser data
  Stg::stg_laser_sample_t* samples = this->lasermodel->GetSamples();
  if(samples)
  {
    // Translate into ROS message format and publish
    Stg::stg_laser_cfg_t cfg = this->lasermodel->GetConfig();
    this->laserMsg.angle_min = -cfg.fov/2.0;
    this->laserMsg.angle_max = +cfg.fov/2.0;
    this->laserMsg.angle_increment = cfg.fov / (double)(cfg.sample_count-1);
    this->laserMsg.range_min = 0.0;
    this->laserMsg.range_max = cfg.range_bounds.max;
    this->laserMsg.ranges.resize(cfg.sample_count);
    this->laserMsg.intensities.resize(cfg.sample_count);
    for(unsigned int i=0;i<cfg.sample_count;i++)
    {
      this->laserMsg.ranges[i] = samples[i].range;
      this->laserMsg.intensities[i] = (uint8_t)samples[i].reflectance;
    }

    this->laserMsg.header.frame_id = "base_laser";
    this->laserMsg.header.stamp = sim_time;
    publish("base_scan",this->laserMsg);
  }
  
  // Also publish the base->base_laser Tx.  This could eventually move
  // into being retrieved from the param server as a static Tx.
  Stg::stg_pose_t lp = this->lasermodel->GetPose();
  tf.sendEuler("base_laser",
               "base",
               lp.x,
               lp.y,
               0.0,
               lp.a,
               0.0,
               0.0,
               sim_time);

  // Get latest odometry data
  // Translate into ROS message format and publish
  this->odomMsg.pos.x = this->positionmodel->est_pose.x;
  this->odomMsg.pos.y = this->positionmodel->est_pose.y;
  this->odomMsg.pos.th = this->positionmodel->est_pose.a;
  Stg::stg_velocity_t v = this->positionmodel->GetVelocity();
  this->odomMsg.vel.x = v.x;
  this->odomMsg.vel.y = v.y;
  this->odomMsg.vel.th = v.a;
  this->odomMsg.stall = this->positionmodel->Stall();
  this->odomMsg.header.frame_id = "odom";
  this->odomMsg.header.stamp = sim_time;
  publish("odom",this->odomMsg);
  tf.sendInverseEuler("odom",
                      "base",
                      odomMsg.pos.x,
                      odomMsg.pos.y,
                      0.0,
                      odomMsg.pos.th,
                      0.0,
                      0.0,
                      sim_time);

  // Also publish the ground truth pose
  Stg::stg_pose_t gpose = this->positionmodel->GetGlobalPose();
  // Use libTF to construct our outgoing message
  libTF::Pose3D pose;
  // Note that we correct for Stage's screwed-up coord system.
  pose.setFromEuler(gpose.y, -gpose.x, 0.0, 
                    Stg::normalize(gpose.a-M_PI/2.0), 0.0, 0.0);

  // FIXME: temp work around during libtf upgrade transition
  std_msgs::Pose3D tmpPose3D = pose.getMessage();
  this->groundTruthMsg.transform.translation.x = tmpPose3D.position.x;
  this->groundTruthMsg.transform.translation.y = tmpPose3D.position.y;
  this->groundTruthMsg.transform.translation.z = tmpPose3D.position.z;
  this->groundTruthMsg.transform.rotation.x    = tmpPose3D.orientation.x;
  this->groundTruthMsg.transform.rotation.y    = tmpPose3D.orientation.y;
  this->groundTruthMsg.transform.rotation.z    = tmpPose3D.orientation.z;
  this->groundTruthMsg.transform.rotation.w    = tmpPose3D.orientation.w;

  this->groundTruthMsg.header.frame_id = "odom";
  this->groundTruthMsg.header.stamp = sim_time;

  publish("base_pose_ground_truth", this->groundTruthMsg);

  this->lock.unlock();
}

int 
main(int argc, char** argv)
{ 
  if( argc < 2 )
  {
    puts(USAGE);
    exit(-1);
  }

  ros::init(argc,argv);

  StageNode sn(argc,argv,argv[1]);

  if(sn.SubscribeModels() != 0)
    exit(-1);

  while(sn.ok() && !sn.world->TestQuit())
  {
    sn.Update();
  }
  
  ros::fini();

  exit(0);
}
