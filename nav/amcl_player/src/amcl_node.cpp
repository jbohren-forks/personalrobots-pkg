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

@b amcl is a probabilistic localization system for a robot moving in
2D.  It implements the adaptive Monte Carlo localization algorithm (as
described by Dieter Fox), which uses a particle filter to track the pose of
a robot against a known map.

This node uses a modified version of the Player @b amcl driver.  For
detailed documentation, consult <a
href="http://playerstage.sourceforge.net/doc/Player-cvs/player/group__driver__amcl.html">Player
amcl documentation</a>.

<hr>

@section usage Usage
@verbatim
$ amcl
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/LaserScan : laser scans.
- @b "initialpose"/Pose2DFloat32: pose used to (re)initialize particle filter

Publishes to (name / type):
- @b "localizedpose"/RobotBase2DOdom : robot's localized map pose.  Only the position information is set (no velocity).
- @b "particlecloud"/ParticleCloud : the set of particles being maintained by the filter.

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
- @b odom_frame_id (string) : The desired frame_id to use for odometery

@todo Expose the various amcl parameters via ROS.

 **/

#include <deque>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "map/map.h"
#include "pf/pf.h"
#include "sensors/amcl_odom.h"
#include "sensors/amcl_laser.h"

#include "ros/assert.h"

// roscpp
#include "ros/node.h"

// Messages that I need
#include "laser_scan/LaserScan.h"
#include "robot_msgs/PoseWithCovariance.h"
#include "robot_msgs/ParticleCloud.h"
#include "robot_msgs/Pose.h"
#include "robot_srvs/StaticMap.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_notifier.h"

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

class AmclNode
{
  public:
    AmclNode();
    ~AmclNode();

    int process();

    int setPose(double x, double y, double a);

  private:
    tf::TransformBroadcaster* tfb_;
    tf::TransformListener* tf_;

    // incoming messages
    robot_msgs::PoseWithCovariance initialPoseMsg_;
    
    // Message callbacks
    void laserReceived(const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& laser_scan);
    void initialPoseReceived();

    double getYaw(robot_msgs::Pose& p);

    //parameter for what odom to use
    std::string odom_frame_id_;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;
    bool have_laser_pose;
    double laser_x, laser_y, laser_yaw;

    tf::MessageNotifier<laser_scan::LaserScan>* laser_scan_notifer;

    // Particle filter
    pf_t *pf_;
    boost::mutex pf_mutex_;
    int pf_min_samples_, pf_max_samples_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;

    AMCLOdom* odom_;
    AMCLLaser* laser_;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    map_t* requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(double& x, double& y, double& yaw, 
                     const ros::Time& t, const std::string& f);

    // buffer of not-yet-transformed scans
    // TODO: use MessageNotifier
    std::deque<laser_scan::LaserScan> laser_scans;
    
    //time for tolerance on the published transform, 
    //basically defines how long a map->odom transform is good for
    double transform_tolerance_;
};

#define USAGE "USAGE: amcl"

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  ros::Node n("amcl");

  AmclNode an;

  n.spin();

  // To quote Morgan, Hooray!
  return(0);
}

AmclNode::AmclNode() :
        map_(NULL),
        have_laser_pose(false),
        pf_(NULL)
{
  // Grab params off the param server
  int max_beams;
  double odom_drift_xx, odom_drift_yy, odom_drift_aa, odom_drift_xa;
  ros::Node::instance()->param("pf_laser_max_beams", max_beams, 20);
  ros::Node::instance()->param("pf_min_samples", pf_min_samples_, 500);
  ros::Node::instance()->param("pf_max_samples", pf_max_samples_, 10000);
  ros::Node::instance()->param("pf_err", pf_err_, 0.01);
  ros::Node::instance()->param("pf_z", pf_z_, 3.0);
  ros::Node::instance()->param("pf_odom_drift_xx", odom_drift_xx, 0.2);
  ros::Node::instance()->param("pf_odom_drift_yy", odom_drift_yy, 0.2);
  ros::Node::instance()->param("pf_odom_drift_aa", odom_drift_aa, 0.2);
  ros::Node::instance()->param("pf_odom_drift_xa", odom_drift_xa, 0.2);
  ros::Node::instance()->param("pf_min_d", d_thresh_, 0.2);
  ros::Node::instance()->param("pf_min_a", a_thresh_, M_PI/6.0);
  ros::Node::instance()->param("pf_odom_frame_id", odom_frame_id_, std::string("odom"));
  ros::Node::instance()->param("pf_transform_tolerance", transform_tolerance_, 0.0);

  double startX, startY, startTH;
  ros::Node::instance()->param("robot_x_start", startX, 0.0);
  ros::Node::instance()->param("robot_y_start", startY, 0.0);
  ros::Node::instance()->param("robot_th_start", startTH, 0.0);

  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster(*ros::Node::instance());
  tf_ = new tf::TransformListener(*ros::Node::instance());

  map_ = requestMap();
  
  // Create the particle filter
  pf_ = pf_alloc(pf_min_samples_, pf_max_samples_);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = startX;
  pf_init_pose_mean.v[1] = startY;
  pf_init_pose_mean.v[2] = startTH;
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = 1.0 * 1.0;
  pf_init_pose_cov.m[1][1] = 1.0 * 1.0;
  pf_init_pose_cov.m[2][2] = 2*M_PI * 2*M_PI;
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  pf_matrix_t drift = pf_matrix_zero();
  drift.m[0][0] = odom_drift_xx;
  drift.m[1][1] = odom_drift_yy;
  drift.m[2][0] = odom_drift_xa;
  drift.m[2][2] = odom_drift_aa;
  odom_ = new AMCLOdom(drift);
  ROS_ASSERT(odom_);
  // Laser
  // We pass in null laser pose to start with; we'll change it later
  pf_vector_t dummy = pf_vector_zero();
  // TODO: expose laser model params
  laser_ = new AMCLLaser(max_beams, 0.1, 0.1, dummy, map_);
  ROS_ASSERT(laser_);

  ros::Node::instance()->advertise<robot_msgs::PoseWithCovariance>("localizedpose",2);
  ros::Node::instance()->advertise<robot_msgs::ParticleCloud>("particlecloud",2);
  laser_scan_notifer = 
          new tf::MessageNotifier<laser_scan::LaserScan>
          (tf_, ros::Node::instance(),  
           boost::bind(&AmclNode::laserReceived, 
                       this, _1), 
           "scan", odom_frame_id_,
           20);
  ros::Node::instance()->subscribe("initialpose", initialPoseMsg_, &AmclNode::initialPoseReceived,this,2);
}

map_t*
AmclNode::requestMap()
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  // get map via RPC
  robot_srvs::StaticMap::Request  req;
  robot_srvs::StaticMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           resp.map.width,
           resp.map.height,
           resp.map.resolution);

  map->size_x = resp.map.width;
  map->size_y = resp.map.height;
  map->scale = resp.map.resolution;
  map->origin_x = resp.map.origin.x;
  map->origin_y = resp.map.origin.y;
  // Convert to player format
  map->cells = new map_cell_t[map->size_x * map->size_y];
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(resp.map.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(resp.map.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}

AmclNode::~AmclNode()
{
  map_free(map_);
  // TODO: delete everything allocated in constructor
}

#if 0
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
    // subtracting base to odom from map to base and send map to odom instead
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      this->tf_->transformPose(odom_frame_id,tf::Stamped<tf::Pose> (btTransform(btQuaternion(pdata->pos.pa, 0, 0), 
                                                                       btVector3(pdata->pos.px, pdata->pos.py, 0.0)).inverse(), 
                                                      t, "base_footprint"),odom_to_map);
    }
    catch(tf::TransformException e){
      ROS_DEBUG("Dropping out of process message step\n");
      return(0);
    }
    
    //we want to send a transform that is good up until a tolerance time so that odom can be used
    ros::Time transform_expiration;
    transform_expiration.fromSec(hdr->timestamp + transform_tolerance_);
    this->tfb_->sendTransform(tf::Stamped<tf::Transform> (tf::Transform(tf::Quaternion( odom_to_map.getRotation() ),
                                                                      tf::Point(      odom_to_map.getOrigin() ) ).inverse(),
                                                        transform_expiration,odom_frame_id, "/map"));

    localizedOdomMsg.pos.x = pdata->pos.px;
    localizedOdomMsg.pos.y = pdata->pos.py;
    localizedOdomMsg.pos.th = pdata->pos.pa;
    localizedOdomMsg.header.stamp.fromSec(hdr->timestamp);
    localizedOdomMsg.header.frame_id = "/map";
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
          tf::PoseTFToMsg(tf::Pose(btQuaternion(resp->particles[i].pose.pa, 0, 0), btVector3(resp->particles[i].pose.px, resp->particles[i].pose.py, 0)),
                          particleCloudMsg.particles[i]);
          
          /*          particleCloudMsg.particles[i].x = resp->particles[i].pose.px;
          particleCloudMsg.particles[i].y = resp->particles[i].pose.py;
          particleCloudMsg.particles[i].th = resp->particles[i].pose.pa;
          */
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
  // Is it a hypothesis message from amcl?
  if(Message::MatchMessage(hdr,
                           PLAYER_MSGTYPE_DATA, 
                           PLAYER_LOCALIZE_DATA_HYPOTHS,
                           this->olocalize_addr))
  {
    // Cast the message payload appropriately 
    player_localize_data_t* pdata = 
            (player_localize_data_t*)data;
    printf("%d hypoths\n", pdata->hypoths_count);
    for(int j=0;j<pdata->hypoths_count;j++)
    {
      printf("%d:  %.3f %.3f %.3f\n", 
             j,
             pdata->hypoths[j].cov[0],
             pdata->hypoths[j].cov[1],
             pdata->hypoths[j].cov[2]);
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
    while(!have_laser_pose)
    {
      ros::Duration d;
      d.fromSec(0.5);
      ROS_INFO("Waiting to receive base->base_laser transform...");
      d.sleep();
    }
    ROS_INFO("Received base->base_laser transform...");
    player_laser_geom_t geom;
    memset(&geom, 0, sizeof(geom));
    geom.pose.px = laser_x;
    geom.pose.py = laser_y;
    geom.pose.pyaw = laser_yaw;
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
#endif

int
AmclNode::setPose(double x, double y, double a)
{
  /*
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
                        */
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
    this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
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
AmclNode::laserReceived(const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& laser_scan)
{
  // Do we have the base->base_laser Tx yet?
  if(!have_laser_pose)
  {
    tf::Stamped<tf::Pose> ident (btTransform(btQuaternion(0,0,0), 
                                             btVector3(0,0,0)), 
                                 ros::Time(), laser_scan->header.frame_id);
    tf::Stamped<btTransform> laser_pose;
    try
    {
      this->tf_->transformPose("base_footprint", ident, laser_pose);
    }
    catch(tf::TransformException e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan->header.frame_id.c_str(),
                "base_footprint");
      return;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.getOrigin().x();
    laser_pose_v.v[1] = laser_pose.getOrigin().y();
    double p,r;
    laser_pose.getBasis().getEulerZYX(laser_pose_v.v[2],p,r);
    laser_->SetLaserPose(laser_pose_v);
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0], 
              laser_pose_v.v[1], 
              laser_pose_v.v[2]);
    have_laser_pose = true;
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if(!getOdomPose(pose.v[0], pose.v[1], pose.v[2], 
                  laser_scan->header.stamp, "base_footprint"))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_mutex_.lock();

  bool update = false;
  pf_vector_t delta;

  if(pf_init_)
  {
    // Compute change in pose
    delta = pf_vector_coord_sub(pose, pf_odom_pose_);

    // See if we should update the filter
    update = fabs(delta.v[0]) > d_thresh_ ||
             fabs(delta.v[1]) > d_thresh_ ||
             fabs(delta.v[2]) > a_thresh_;
  }

  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    update = true;
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && update)
  {
    //printf("pose\n");
    //pf_vector_fprintf(pose, stdout, "%.3f");

    AMCLOdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;

    // Use the action data to update the filter
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);

    // Pose at last filter update
    //this->pf_odom_pose = pose;
  }

  // If the robot has moved, update the filter
  if(update)
  {
    AMCLLaserData ldata;
    ldata.sensor = laser_;
    ldata.range_count = laser_scan->ranges.size();
    ldata.range_max = laser_scan->range_max;
    // The AMCLLaserData destructor will free this memory
    ldata.ranges = new double[ldata.range_count][2];
    ROS_ASSERT(ldata.ranges);
    for(int i=0;i<ldata.range_count;i++)
    {
      // amcl doesn't (yet) have a concept of min range.  So we'll map short
      // readings to max range.
      if(laser_scan->ranges[i] <= laser_scan->range_min)
        ldata.ranges[i][0] = laser_scan->range_max;
      else
        ldata.ranges[i][0] = laser_scan->ranges[i];
      // Compute bearing
      ldata.ranges[i][1] = laser_scan->angle_min + 
              (i * laser_scan->angle_increment);
    }

    laser_->UpdateSensor(pf_, (AMCLSensorData*)&ldata);
    pf_odom_pose_ = pose;
  
    // Resample the particles
    pf_update_resample(pf_);
    ROS_INFO("Num samples: %d\n", pf_->sets[pf_->current_set].sample_count);

    // Read out the current hypotheses
    double max_weight = 0.0;
    pf_vector_t max_weight_pose={{0.0,0.0,0.0}};
    const int MAX_HYPS = 8;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(MAX_HYPS);
    int hyp_count = 0;
    for(hyp_count = 0; hyp_count < MAX_HYPS; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
        break;

      //pf_vector_fprintf(pose_mean, stdout, "%.3f");

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if(hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_pose = hyps[hyp_count].pf_pose_mean;
      }
    }

    if(max_weight > 0.0)
    {
      ROS_INFO("Max weight pose: %.3f %.3f %.3f",
               max_weight_pose.v[0],
               max_weight_pose.v[1],
               max_weight_pose.v[2]);
    }
    else
    {
      ROS_ERROR("No pose!");
    }
  }

#if 0

  double timestamp = scan.header.stamp.toSec();
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
#endif
  pf_mutex_.unlock();
}

double
AmclNode::getYaw(robot_msgs::Pose& p)
{
  tf::Stamped<tf::Transform> t;
  tf::PoseMsgToTF(p, t);
  double yaw, pitch, roll;
  btMatrix3x3 mat = t.getBasis();
  mat.getEulerZYX(yaw,pitch,roll);
  return yaw;
}

void 
AmclNode::initialPoseReceived()
{
  ROS_INFO("Setting pose: %.3f %.3f %.3f", 
           initialPoseMsg_.pose.position.x,
           initialPoseMsg_.pose.position.y,
           getYaw(initialPoseMsg_.pose));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = initialPoseMsg_.pose.position.x;
  pf_init_pose_mean.v[1] = initialPoseMsg_.pose.position.y;
  pf_init_pose_mean.v[2] = getYaw(initialPoseMsg_.pose);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = 1.0 * 1.0;
  pf_init_pose_cov.m[1][1] = 1.0 * 1.0;
  pf_init_pose_cov.m[2][2] = 2*M_PI * 2*M_PI;
  pf_mutex_.lock();
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;
  pf_mutex_.unlock();
}
