/*
 * amcl_player
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**

@mainpage

@htmlinclude manifest.html

@b amcl_player is a probabilistic localization system for a robot moving in
2D.  It implements the adaptive Monte Carlo localization algorithm (as
described by Dieter Fox), which uses a particle filter to track the pose of
a robot against a known map.

This node wraps up the Player @b amcl driver.  For detailed documentation,
consult <a href="http://playerstage.sourceforge.net/doc/Player-cvs/player/group__driver__amcl.html">Player amcl documentation</a>.

<hr>

@section usage Usage
@verbatim
$ amcl_player
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "odom"/RobotBase2DOdom : robot's odometric pose.  Only the position information is used (velocity is ignored).
- @b "scan"/LaserScan : laser scans.
- @b "initialpose"/Pose2DFloat32: pose used to (re)initialize particle filter

Publishes to (name / type):
- @b "localizedpose"/RobotBase2DOdom : robot's localized map pose.  Only the position information is set (no velocity).
- @b "particlecloud"/ParticleCloud2D : the set of particles being maintained by the filter.

<hr>

@section parameters ROS parameters

- @b "robot_x_start" (double) : The starting X position of the robot, default: 0.
- @b "robot_y_start" (double) : The starting Y position of the robot, default: 0.
- @b "robot_th_start" (double) : The starting TH position of the robot, default: 0.
- @b pf_laser_max_beams (int) : The number of laser beams to use when localizing, default: 20.
- @b pf_min_samples (int) : The minimum number of particles used when localizing, default: 500
- @b pf_max_samples (int) : The maximum number of particles used when localizing, default: 10000
- @b pf_odom_drift_xx (double) : Element 0,0 of the covariance matrix used in estimating odometric error, default: 0.2
- @b pf_odom_drift_yy (double) : Element 1,1 of the covariance matrix used in estimating odometric error, default: 0.2
- @b pf_odom_drift_aa (double) : Element 2,2 of the covariance matrix used in estimating odometric error, default: 0.2
- @b pf_odom_drift_xa (double) : Element 2,0 of the covariance matrix used in estimating odometric error, default: 0.2
- @b pf_min_d (double) : Minimum translational change (meters) required to trigger filter update, default: 0.2
- @b pf_min_a (double) : Minimum rotational change (radians) required to trigger filter update, default: pi/6.0

@todo Expose the various amcl parameters via ROS.

 **/

#include <deque>

#include "rosconsole/rosassert.h"

// For core Player stuff (message queues, config file objects, etc.)
#include "libplayercore/playercore.h"
// TODO: remove XDR dependency
#include "libplayerxdr/playerxdr.h"

// roscpp
#include "ros/node.h"

// Messages that I need
#include "std_msgs/LaserScan.h"
#include "std_msgs/RobotBase2DOdom.h"
#include "std_msgs/ParticleCloud2D.h"
#include "std_msgs/Pose2DFloat32.h"
#include "std_srvs/StaticMap.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
// check that given coords are valid (i.e., on the map)
#define MAP_VALID(mf, i, j) ((i >= 0) && (i < mf->sx) && (j >= 0) && (j < mf->sy))

const int PLAYER_QUEUE_LEN = 32;

// Must prototype this function here.  It's implemented inside
// libplayerdrivers.
Driver* AdaptiveMCL_Init(ConfigFile* cf, int section);

class AmclNode: public ros::node, public Driver
{
  public:
    AmclNode();
    ~AmclNode();

    int Setup() {return(0);}
    int Shutdown() {return(0);}
    int ProcessMessage(QueuePointer &resp_queue, 
                       player_msghdr * hdr,
                       void * data);

    int start();
    int stop();

    int process();

    int setPose(double x, double y, double a);

  private:
    tf::TransformBroadcaster* tf;
    tf::TransformListener* tfL;

    ConfigFile* cf;

    // incoming messages
    std_msgs::RobotBase2DOdom localizedOdomMsg;
    std_msgs::ParticleCloud2D particleCloudMsg;
    std_msgs::RobotBase2DOdom odomMsg;
    std_msgs::LaserScan laserMsg;
    std_msgs::Pose2DFloat32 initialPoseMsg;
    
    // Message callbacks
    void odomReceived();
    void laserReceived();
    void initialPoseReceived();

    // These are the devices that amcl offers, and to which we subscribe
    Driver* driver;
    Device* pdevice;
    Device* ldevice;
    player_devaddr_t oposition2d_addr;
    player_devaddr_t olocalize_addr;

    // These are the devices that amcl requires, and so which we must
    // provide
    player_devaddr_t position2d_addr;
    player_devaddr_t laser_addr;
    player_devaddr_t map_addr;
    Device* position2d_dev;
    Device* laser_dev;
    Device* map_dev;

    char* mapdata;
    int sx, sy;
    double resolution;
    // static laser transform (todo: make this cleaner)
    double laser_x_offset;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    // Helper to get odometric pose from transform system
    bool getOdomPose(double& x, double& y, double& yaw, 
                     const ros::Time& t, const std::string& f);

    // buffer of not-yet-transformed scans
    std::deque<std_msgs::LaserScan> laser_scans;
};

#define USAGE "USAGE: amcl_player"

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  AmclNode an;

  // Start up the robot
  //if(an.start() != 0)
    //exit(-1);

  /////////////////////////////////////////////////////////////////
  // Main loop; grab messages off our queue and republish them via ROS
  while(an.ok())
    an.process();
  /////////////////////////////////////////////////////////////////

  // Stop the robot
  an.stop();

  ros::fini();

  // To quote Morgan, Hooray!
  return(0);
}

AmclNode::AmclNode() :
        ros::node("amcl_player"), 
        Driver(NULL,-1,false,PLAYER_QUEUE_LEN)
        //tfClient(*this)
{
  // libplayercore boiler plate
  player_globals_init();
  itable_init();

  // TODO: remove XDR dependency
  playerxdr_ftable_init();

  // get map via RPC
  std_srvs::StaticMap::request  req;
  std_srvs::StaticMap::response resp;
  puts("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    puts("request failed; trying again...");
    usleep(1000000);
  }
  printf("Received a %d X %d map @ %.3f m/pix\n",
         resp.map.width,
         resp.map.height,
         resp.map.resolution);

  this->sx = resp.map.width;
  this->sy = resp.map.height;
  this->resolution = resp.map.resolution;
  // Convert to player format
  this->mapdata = new char[this->sx*this->sy];
  for(int i=0;i<this->sx*this->sy;i++)
  {
    if(resp.map.data[i] == 0)
      this->mapdata[i] = -1;
    else if(resp.map.data[i] == 100)
      this->mapdata[i] = +1;
    else
      this->mapdata[i] = 0;
  }

  /// @todo Find a way to make this work for pr2, with mechanism control,
  /// but not break for STAIR.  Somebody needs to be periodically
  /// publishing the base->base_laser Tx.  Or else we need a more standard
  /// way of retrieving such Txs;
  //
  // retrieve static laser transform, if it's available
  param("laser_x_offset", laser_x_offset, 0.0);
  
  //assert(read_map_from_image(&this->sx, &this->sy, &this->mapdata, fname, 0)
         //== 0);

  // TODO: automatically convert between string and player_devaddr_t
  // representations

  // The Player address that will be assigned to this device.  The format
  // is interface:index.  The interface must match what the driver is
  // expecting to provide.  The value of the index doesn't really matter, 
  // but 0 is most common.
  const char* oposition2d_saddr = "position2d:0";
  this->oposition2d_addr.host = 0;
  this->oposition2d_addr.robot = 0;
  this->oposition2d_addr.interf = PLAYER_POSITION2D_CODE;
  this->oposition2d_addr.index = 0;

  const char* olocalize_saddr = "localize:0";
  this->olocalize_addr.host = 0;
  this->olocalize_addr.robot = 0;
  this->olocalize_addr.interf = PLAYER_LOCALIZE_CODE;
  this->olocalize_addr.index = 0;

  const char* position2d_saddr = "odometry:::position2d:1";
  this->position2d_addr.host = 0;
  this->position2d_addr.robot = 0;
  this->position2d_addr.interf = PLAYER_POSITION2D_CODE;
  this->position2d_addr.index = 1;

  const char* laser_saddr = "laser:0";
  this->laser_addr.host = 0;
  this->laser_addr.robot = 0;
  this->laser_addr.interf = PLAYER_LASER_CODE;
  this->laser_addr.index = 0;

  const char* map_saddr = "laser:::map:0";
  this->map_addr.host = 0;
  this->map_addr.robot = 0;
  this->map_addr.interf = PLAYER_MAP_CODE;
  this->map_addr.index = 0;

  this->position2d_dev = deviceTable->AddDevice(this->position2d_addr, 
                                                NULL, false);
  ROS_ASSERT(this->position2d_dev);
  this->position2d_dev->InQueue = QueuePointer(this->Driver::InQueue);
  this->position2d_dev->driver = (Driver*)this;

  this->laser_dev = deviceTable->AddDevice(this->laser_addr, 
                                           NULL, false);
  ROS_ASSERT(this->laser_dev);
  this->laser_dev->InQueue = QueuePointer(this->Driver::InQueue);
  this->laser_dev->driver = (Driver*)this;

  this->map_dev = deviceTable->AddDevice(this->map_addr, 
                                         NULL, false);
  ROS_ASSERT(this->map_dev);
  this->map_dev->InQueue = QueuePointer(this->Driver::InQueue);
  this->map_dev->driver = (Driver*)this;

  // Create a ConfigFile object, into which we'll stuff parameters.
  // Drivers assume that this object will persist throughout execution
  // (e.g., they store pointers to data inside it).  So it must NOT be
  // deleted until after the driver is shut down.
  this->cf = new ConfigFile();

  // Insert (name,value) pairs into the ConfigFile object.  These would
  // presumably come from the param server
  this->cf->InsertFieldValue(0,"provides",oposition2d_saddr);
  this->cf->InsertFieldValue(1,"provides",olocalize_saddr);

  // Fill in the requires fields for the device that this device
  // subscribes to
  this->cf->InsertFieldValue(0,"requires",position2d_saddr);
  this->cf->InsertFieldValue(1,"requires",laser_saddr);
  this->cf->InsertFieldValue(2,"requires",map_saddr);

  // Turn this on for serious AMCL debugging, but you must have built
  // player with ENABLE_RTKGUI=ON.
  this->cf->InsertFieldValue(0,"enable_gui","0");

  // Grab params off the param server
  int max_beams, min_samples, max_samples;
  double odom_drift_xx, odom_drift_yy, odom_drift_aa, odom_drift_xa;
  double d_thresh, a_thresh;
  param("pf_laser_max_beams", max_beams, 20);
  param("pf_min_samples", min_samples, 500);
  param("pf_max_samples", max_samples, 10000);
  param("pf_odom_drift_xx", odom_drift_xx, 0.2);
  param("pf_odom_drift_yy", odom_drift_yy, 0.2);
  param("pf_odom_drift_aa", odom_drift_aa, 0.2);
  param("pf_odom_drift_xa", odom_drift_xa, 0.2);
  param("pf_min_d", d_thresh, 0.2);
  param("pf_min_a", a_thresh, M_PI/6.0);
  // Annoyingly, we have to convert them back to strings for insertion into
  // Player's config file object
  char valbuf[1024];
  snprintf(valbuf,sizeof(valbuf),"%d",max_beams);
  this->cf->InsertFieldValue(0,"laser_max_beams",valbuf);
  snprintf(valbuf,sizeof(valbuf),"%d",min_samples);
  this->cf->InsertFieldValue(0,"pf_min_samples",valbuf);
  snprintf(valbuf,sizeof(valbuf),"%d",max_samples);
  this->cf->InsertFieldValue(0,"pf_max_samples",valbuf);

  snprintf(valbuf,sizeof(valbuf),"%.3f",d_thresh);
  this->cf->InsertFieldValue(0,"update_thresh",valbuf);
  snprintf(valbuf,sizeof(valbuf),"%.3f",a_thresh);
  this->cf->InsertFieldValue(1,"update_thresh",valbuf);

  snprintf(valbuf,sizeof(valbuf),"%.3f",odom_drift_xx);
  this->cf->InsertFieldValue(0,"odom_drift[0]",valbuf);
  snprintf(valbuf,sizeof(valbuf),"%.3f",0.0);
  this->cf->InsertFieldValue(1,"odom_drift[0]",valbuf);
  this->cf->InsertFieldValue(2,"odom_drift[0]",valbuf);

  snprintf(valbuf,sizeof(valbuf),"%.3f",0.0);
  this->cf->InsertFieldValue(0,"odom_drift[1]",valbuf);
  snprintf(valbuf,sizeof(valbuf),"%.3f",odom_drift_yy);
  this->cf->InsertFieldValue(1,"odom_drift[1]",valbuf);
  snprintf(valbuf,sizeof(valbuf),"%.3f",0.0);
  this->cf->InsertFieldValue(2,"odom_drift[1]",valbuf);

  snprintf(valbuf,sizeof(valbuf),"%.3f",odom_drift_xa);
  this->cf->InsertFieldValue(0,"odom_drift[2]",valbuf);
  snprintf(valbuf,sizeof(valbuf),"%.3f",0.0);
  this->cf->InsertFieldValue(1,"odom_drift[2]",valbuf);
  snprintf(valbuf,sizeof(valbuf),"%.3f",odom_drift_aa);
  this->cf->InsertFieldValue(2,"odom_drift[2]",valbuf);

  // Options
  //this->cf->InsertFieldValue(0,"enable_gui","1");
  //this->cf->InsertFieldValue(0,"odom_drift[0]","0.001");
  //this->cf->InsertFieldValue(1,"odom_drift[1]","0.001");
  //this->cf->InsertFieldValue(0,"odom_drift[2]","0.001");
  //this->cf->InsertFieldValue(2,"odom_drift[2]","0.001");
  //this->cf->InsertFieldValue(0,"update_thresh","10.0");
  //this->cf->InsertFieldValue(1,"update_thresh","180.0");

  // Create an instance of the driver, passing it the ConfigFile object.
  // The -1 tells it to look into the "global" section of the ConfigFile,
  // which is where ConfigFile::InsertFieldValue() put the parameters.
  ROS_ASSERT((this->driver = AdaptiveMCL_Init(cf, -1)));

  // Print out warnings about parameters that were set, but which the
  // driver never looked at.
  cf->WarnUnused();

  // Grab from the global deviceTable a pointer to the Device that was 
  // created as part of the driver's initialization.
  ROS_ASSERT((this->pdevice = deviceTable->GetDevice(oposition2d_saddr,false)));
  ROS_ASSERT((this->ldevice = deviceTable->GetDevice(olocalize_addr,false)));

  double startX, startY, startTH;
  param("robot_x_start", startX, 0.0);
  param("robot_y_start", startY, 0.0);
  param("robot_th_start", startTH, 0.0);
  this->setPose(startX, startY, startTH);

  cloud_pub_interval.fromSec(1.0);

  start();

  // Give Player a chance to start up, under heavy load conditions.
  // TODO: remove this.
  usleep(5000000);

  this->tf = new tf::TransformBroadcaster(*this);
  this->tfL = new tf::TransformListener(*this, true, 
                                        10000000000ULL,
                                          200000000ULL);
                                         

  advertise<std_msgs::RobotBase2DOdom>("localizedpose",2);
  advertise<std_msgs::ParticleCloud2D>("particlecloud",2);
  //subscribe("odom", odomMsg, &AmclNode::odomReceived,2);
  subscribe("scan", laserMsg, &AmclNode::laserReceived,2);
  subscribe("initialpose", initialPoseMsg, &AmclNode::initialPoseReceived,2);
}

AmclNode::~AmclNode()
{
  delete this->cf;
  player_globals_fini();
  free(this->mapdata);
}

int 
AmclNode::ProcessMessage(QueuePointer &resp_queue, 
                         player_msghdr * hdr,
                         void * data)
{
  // Is it a new pose from amcl?
  if(Message::MatchMessage(hdr,
                           PLAYER_MSGTYPE_DATA, 
                           PLAYER_POSITION2D_DATA_STATE,
                           this->oposition2d_addr))
  {
    // Cast the message payload appropriately 
    player_position2d_data_t* pdata = 
            (player_position2d_data_t*)data;

    
    // publish new transform robot->map
    ros::Time t;
    t.fromSec(hdr->timestamp);
    /*
    this->tf->sendTransform(tf::Stamped<tf::Transform> (tf::Transform(tf::Quaternion(pdata->pos.pa, 0, 0), 
                                                                      tf::Point(pdata->pos.px, pdata->pos.py, 0.0)),
                                                        t, "base","map"));
                                                        */
    this->tf->sendTransform(tf::Stamped<tf::Transform> (tf::Transform(tf::Quaternion(pdata->pos.pa, 0, 0), 
                                                                      tf::Point(pdata->pos.px, pdata->pos.py, 0.0)).inverse(),
                                                        t, "map", "base"));

    /*
    printf("lpose: (%.3f %.3f %.3f) @ (%llu:%llu)\n",
           pdata->pos.px,
           pdata->pos.py,
           RTOD(pdata->pos.pa),
           (long long unsigned int)floor(hdr->timestamp),
           (long long unsigned int)((hdr->timestamp - floor(hdr->timestamp)) * 
                          1000000000ULL));
                          */
    
    localizedOdomMsg.pos.x = pdata->pos.px;
    localizedOdomMsg.pos.y = pdata->pos.py;
    localizedOdomMsg.pos.th = pdata->pos.pa;
    localizedOdomMsg.header.stamp.fromSec(hdr->timestamp);
    try
    {
	localizedOdomMsg.header.frame_id = "map";
    }
    catch(...)
    {
      // WTF is this?
      printf("Somehow could not set frame_id to map\n");
    }
    /*
    printf("O: %.6f %.3f %.3f %.3f\n",
           hdr->timestamp, 
           localizedOdomMsg.pos.x, 
           localizedOdomMsg.pos.y, 
           localizedOdomMsg.pos.th);
           */
    publish("localizedpose", localizedOdomMsg);

    if((ros::Time::now() - this->last_cloud_pub_time) >= cloud_pub_interval)
    {
      last_cloud_pub_time = ros::Time::now();
      // Also request and publish the particle cloud
      Message* msg;
      if((msg = this->ldevice->Request(this->Driver::InQueue,
                                       PLAYER_MSGTYPE_REQ,
                                       PLAYER_LOCALIZE_REQ_GET_PARTICLES,
                                       NULL, 0, NULL, true)))
      {
        player_localize_get_particles_t* resp =
                (player_localize_get_particles_t*)(msg->GetPayload());
        particleCloudMsg.set_particles_size(resp->particles_count);
        for(unsigned int i=0;i<resp->particles_count;i++)
        {
          particleCloudMsg.particles[i].x = resp->particles[i].pose.px;
          particleCloudMsg.particles[i].y = resp->particles[i].pose.py;
          particleCloudMsg.particles[i].th = resp->particles[i].pose.pa;
        }
        publish("particlecloud", particleCloudMsg);
        delete msg;
      }
      else
      {
        puts("Warning: failed to get particle cloud from amcl");
      }
    }

    return(0);
  }
  // Is it a request for the map metadata?
  else if(Message::MatchMessage(hdr,
                                PLAYER_MSGTYPE_REQ, 
                                PLAYER_MAP_REQ_GET_INFO,
                                this->map_addr))
  {
    player_map_info_t info;
    info.scale = this->resolution;
    info.width = this->sx;
    info.height = this->sy;
    info.origin.px = 0;
    info.origin.py = 0;
    info.origin.pa = 0;
    this->Publish(this->map_addr, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_MAP_REQ_GET_INFO,
                  (void*)&info);
    return(0);
  }
  // Is it a request for a map tile?
  else if(Message::MatchMessage(hdr,
                                PLAYER_MSGTYPE_REQ, 
                                PLAYER_MAP_REQ_GET_DATA,
                                this->map_addr))
  {
    player_map_data_t* mapreq = (player_map_data_t*)data;

    player_map_data_t mapresp;

    int i, j;
    int oi, oj, si, sj;

    // Construct reply
    oi = mapresp.col = mapreq->col;
    oj = mapresp.row = mapreq->row;
    si = mapresp.width = mapreq->width;
    sj = mapresp.height = mapreq->height;
    mapresp.data_count = mapresp.width * mapresp.height;
    mapresp.data = new int8_t [mapresp.data_count];
    ROS_ASSERT(mapresp.data);
    // Grab the pixels from the map
    for(j = 0; j < sj; j++)
    {
      for(i = 0; i < si; i++)
      {
        if(MAP_VALID(this, i + oi, j + oj))
          mapresp.data[i + j * si] = this->mapdata[MAP_IDX(this->sx, i+oi, j+oj)];
        else
        {
          PLAYER_WARN2("requested cell (%d,%d) is offmap", i+oi, j+oj);
          mapresp.data[i + j * si] = 0;
        }
      }
    }

    this->Publish(this->map_addr, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_MAP_REQ_GET_DATA,
                  (void*)&mapresp);
    delete [] mapresp.data;
    return(0);
  }
  // Is it a request for laser geometry (pose of laser wrt parent)?
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_LASER_REQ_GET_GEOM,
                                 this->laser_addr))
  {
    player_laser_geom_t geom;
    memset(&geom, 0, sizeof(geom));
    geom.pose.px = laser_x_offset;
    geom.pose.py = 0;
    geom.pose.pyaw = 0;
    geom.size.sl = 0.06;
    geom.size.sw = 0.06;

    this->Publish(this->laser_addr,
                  resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_LASER_REQ_GET_GEOM,
                  (void*)&geom);
    return(0);
  }
  // Is it an ACK from a request that I sent earlier?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_RESP_ACK,
                                 -1))
    return(0);
  else
  {
    printf("Unhandled Player message %d:%d:%d:%d\n",
           hdr->type,
           hdr->subtype,
           hdr->addr.interf,
           hdr->addr.index);
    return(-1);
  }
}

int 
AmclNode::start()
{
  // Subscribe to device, which causes it to startup
  if(this->pdevice->Subscribe(this->Driver::InQueue) != 0)
  {
    puts("Failed to subscribe the driver");
    return(-1);
  }
  else
  {
    //this->setPose(0,0,0);
    return(0);
  }
}

int 
AmclNode::stop()
{
  // Unsubscribe from the device, which causes it to shutdown
  if(pdevice->Unsubscribe(this->Driver::InQueue) != 0)
  {
    puts("Failed to stop the driver");
  }
  else
  {
    // Give the driver a chance to shutdown.  Wish there were a way to
    // detect when that happened.
    usleep(1000000);
  }
  return(0);
}

int 
AmclNode::process()
{
  // Can't block here, because we won't exit cleanly.  The Wait() call
  // blocks on pthread_cond_wait(), which is a cancellation point, but in
  // this case we're the main thread and noone will try to cancel us.
  //
  // Block until there's a message on our queue
  //this->Driver::InQueue->Wait();

  if(!this->Driver::InQueue->Empty())
    this->Driver::ProcessMessages();
  else
    usleep(100000);

  return(0);
}

int
AmclNode::setPose(double x, double y, double a)
{
  player_localize_set_pose_t p;

  p.mean.px = x;
  p.mean.py = y;
  p.mean.pa = a;

  p.cov[0] = 0.25*0.25;
  p.cov[1] = 0.25*0.25;
  p.cov[2] = (M_PI/12.0)*(M_PI/12.0);

  this->ldevice->PutMsg(this->Driver::InQueue,
                        PLAYER_MSGTYPE_REQ,
                        PLAYER_LOCALIZE_REQ_SET_POSE,
                        (void*)&p,0,NULL);
  return(0);
}

bool
AmclNode::getOdomPose(double& x, double& y, double& yaw, 
                      const ros::Time& t, const std::string& f)
{
  // Get the robot's pose 
  tf::Stamped<tf::Pose> ident (btTransform(btQuaternion(0,0,0), 
                                           btVector3(0,0,0)), t, f);
  tf::Stamped<btTransform> odom_pose;
  try
  {
    this->tfL->transformPose("odom", ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    //ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch,roll;
  odom_pose.getBasis().getEulerZYX(yaw, pitch, roll);

  return true;
}

void
AmclNode::laserReceived()
{
  // Put it on the queue
  std_msgs::LaserScan newscan(laserMsg);
  laser_scans.push_back(newscan);

  // Process the queued scans
  while(!laser_scans.empty())
  {
    std_msgs::LaserScan scan = laser_scans.front();
    
    // Where was the robot when this scan was taken?
    double x, y, yaw;
    if(!getOdomPose(x, y, yaw, scan.header.stamp, scan.header.frame_id))
      break;

    laser_scans.pop_front();

    double timestamp = scan.header.stamp.to_double();
    //printf("I: %.6f %.3f %.3f %.3f\n",
           //timestamp, x, y, yaw);

    // Synthesize an odometry message
    player_position2d_data_t pdata_odom;
    pdata_odom.pos.px = x;
    pdata_odom.pos.py = y;
    pdata_odom.pos.pa = yaw;
    pdata_odom.vel.px = 0.0;
    pdata_odom.vel.py = 0.0;
    pdata_odom.vel.pa = 0.0;
    pdata_odom.stall = 0;

    this->Driver::Publish(this->position2d_addr,
                          PLAYER_MSGTYPE_DATA,
                          PLAYER_POSITION2D_DATA_STATE,
                          (void*)&pdata_odom,0,
                          &timestamp);


    // Got new scan; reformat and pass it on
    player_laser_data_t pdata;
    pdata.min_angle = scan.angle_min;
    pdata.max_angle = scan.angle_max;
    pdata.resolution = scan.angle_increment;
    // HACK, until the hokuyourg_player node is fixed
    if(scan.range_max > 0.1)
      pdata.max_range = scan.range_max;
    else
      pdata.max_range = 30.0;
    pdata.ranges_count = scan.get_ranges_size();
    pdata.ranges = new float[pdata.ranges_count];
    ROS_ASSERT(pdata.ranges);
    // We have to iterate over, rather than block copy, the ranges, because
    // we have to filter out short readings.
    for(unsigned int i=0;i<pdata.ranges_count;i++)
    {
      // Player doesn't have a concept of min range.  So we'll map short
      // readings to max range.
      if(scan.ranges[i] <= scan.range_min)
        pdata.ranges[i] = scan.range_max;
      else
        pdata.ranges[i] = scan.ranges[i];
    }
    pdata.intensity_count = scan.get_intensities_size();
    pdata.intensity = new uint8_t[pdata.intensity_count];
    ROS_ASSERT(pdata.intensity);
    memset(pdata.intensity,0,sizeof(uint8_t)*pdata.intensity_count);
    pdata.id = scan.header.seq;

    this->Driver::Publish(this->laser_addr,
                          PLAYER_MSGTYPE_DATA,
                          PLAYER_LASER_DATA_SCAN,
                          (void*)&pdata,0,
                          &timestamp);

    delete[] pdata.ranges;
    delete[] pdata.intensity;
  }
}

void 
AmclNode::initialPoseReceived()
{
  this->AmclNode::setPose(this->initialPoseMsg.x,
                          this->initialPoseMsg.y,
                          this->initialPoseMsg.th);
}

void
AmclNode::odomReceived()
{
  // Got new odom; reformat and pass it on
  player_position2d_data_t pdata;
  pdata.pos.px = this->odomMsg.pos.x;
  pdata.pos.py = this->odomMsg.pos.y;
  pdata.pos.pa = this->odomMsg.pos.th;
  pdata.vel.px = this->odomMsg.vel.x;
  pdata.vel.py = this->odomMsg.vel.y;
  pdata.vel.pa = this->odomMsg.vel.th;
  pdata.stall = this->odomMsg.stall;

  double timestamp = this->odomMsg.header.stamp.to_double();

  this->Driver::Publish(this->position2d_addr,
                        PLAYER_MSGTYPE_DATA,
                        PLAYER_POSITION2D_DATA_STATE,
                        (void*)&pdata,0,
                        &timestamp);

  /*
  printf("opose: (%.3f %.3f %.3f) @ (%llu:%llu)\n",
         pdata.pos.px,
         pdata.pos.py,
         RTOD(pdata.pos.pa),
         (long long unsigned int)floor(timestamp),
         (long long unsigned int)((timestamp - floor(timestamp)) * 
                                  1000000000ULL));
                                  */
}

