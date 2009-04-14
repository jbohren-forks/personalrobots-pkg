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
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_notifier.h"

using namespace amcl;

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

  private:
    tf::TransformBroadcaster* tfb_;
    tf::TransformListener* tf_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);

    // incoming messages
    robot_msgs::PoseWithCovariance initial_pose_;
    
    // Message callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
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

    tf::MessageNotifier<laser_scan::LaserScan>* laser_scan_notifer;

    // Particle filter
    pf_t *pf_;
    boost::mutex pf_mutex_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;

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
    ros::Duration transform_tolerance_;
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
        latest_tf_valid_(false),
        map_(NULL),
        have_laser_pose(false),
        pf_(NULL),
        resample_count_(0)
{
  // Grab params off the param server
  int max_beams, min_particles, max_particles;
  double alpha1, alpha2, alpha3, alpha4;
  double alpha_slow, alpha_fast;
  double z_hit, z_short, z_max, z_rand, sigma_hit, lambda_short;
  double pf_err, pf_z;
  ros::Node::instance()->param("~laser_max_beams", max_beams, 30);
  ros::Node::instance()->param("~min_particles", min_particles, 100);
  ros::Node::instance()->param("~max_particles", max_particles, 5000);
  ros::Node::instance()->param("~kld_err", pf_err, 0.01);
  ros::Node::instance()->param("~kld_z", pf_z, 0.99);
  ros::Node::instance()->param("~odom_alpha1", alpha1, 0.2);
  ros::Node::instance()->param("~odom_alpha2", alpha2, 0.2);
  ros::Node::instance()->param("~odom_alpha3", alpha3, 0.2);
  ros::Node::instance()->param("~odom_alpha4", alpha4, 0.2);

  ros::Node::instance()->param("~laser_z_hit", z_hit, 0.95);
  ros::Node::instance()->param("~laser_z_short", z_short, 0.1);
  ros::Node::instance()->param("~laser_z_max", z_max, 0.05);
  ros::Node::instance()->param("~laser_z_rand", z_rand, 0.05);
  ros::Node::instance()->param("~laser_sigma_hit", sigma_hit, 0.2);
  ros::Node::instance()->param("~laser_lambda_short", lambda_short, 0.1);
  double laser_likelihood_max_dist;
  ros::Node::instance()->param("~laser_likelihood_max_dist", 
                               laser_likelihood_max_dist, 2.0);
  std::string tmp_model_type;
  laser_model_t laser_model_type;
  ros::Node::instance()->param("~laser_model_type", tmp_model_type, std::string("beam"));
  if(tmp_model_type == "beam")
    laser_model_type = LASER_MODEL_BEAM;
  else if(tmp_model_type == "likelihood_field")
    laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  else
  {
    ROS_WARN("Unknown laser model type \"%s\"; defaulting to beam model",
             tmp_model_type.c_str());
    laser_model_type = LASER_MODEL_BEAM;
  }

  ros::Node::instance()->param("~update_min_d", d_thresh_, 0.2);
  ros::Node::instance()->param("~update_min_a", a_thresh_, M_PI/6.0);
  ros::Node::instance()->param("~odom_frame_id", odom_frame_id_, std::string("odom"));
  ros::Node::instance()->param("~resample_interval", resample_interval_, 10);
  double tmp_tol;
  ros::Node::instance()->param("~transform_tolerance", tmp_tol, 0.1);
  ros::Node::instance()->param("~recovery_alpha_slow", alpha_slow, 0.001);
  ros::Node::instance()->param("~recovery_alpha_fast", alpha_fast, 0.1);


  transform_tolerance_.fromSec(tmp_tol);

  double startX, startY, startTH;
  ros::Node::instance()->param("~initial_pose_x", startX, 0.0);
  ros::Node::instance()->param("~initial_pose_y", startY, 0.0);
  ros::Node::instance()->param("~initial_pose_a", startTH, 0.0);

  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster(*ros::Node::instance());
  tf_ = new tf::TransformListener(*ros::Node::instance());

  map_ = requestMap();
  
  // Create the particle filter
  pf_ = pf_alloc(min_particles, max_particles,
                 alpha_slow, alpha_fast,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator, 
                 (void *)map_);
  pf_->pop_err = pf_err;
  pf_->pop_z = pf_z;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = startX;
  pf_init_pose_mean.v[1] = startY;
  pf_init_pose_mean.v[2] = startTH;
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = 0.5 * 0.5;
  pf_init_pose_cov.m[1][1] = 0.5 * 0.5;
  pf_init_pose_cov.m[2][2] = M_PI/12.0 * M_PI/12.0;
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  odom_ = new AMCLOdom(alpha1, alpha2, alpha3, alpha4);
  ROS_ASSERT(odom_);
  // Laser
  laser_ = new AMCLLaser(max_beams, map_);
  ROS_ASSERT(laser_);
  if(laser_model_type == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit, z_short, z_max, z_rand, 
                         sigma_hit, lambda_short, 0.0);
  else
    laser_->SetModelLikelihoodField(z_hit, z_rand, sigma_hit, 
                                    laser_likelihood_max_dist);

  ros::Node::instance()->advertise<robot_msgs::PoseWithCovariance>("amcl_pose",2);
  ros::Node::instance()->advertise<robot_msgs::ParticleCloud>("particlecloud",2);
  ros::Node::instance()->advertiseService("global_localization",
                                          &AmclNode::globalLocalizationCallback,
                                          this);
  laser_scan_notifer = 
          new tf::MessageNotifier<laser_scan::LaserScan>
          (tf_, ros::Node::instance(),  
           boost::bind(&AmclNode::laserReceived, 
                       this, _1), 
           "scan", odom_frame_id_,
           100);
  ros::Node::instance()->subscribe("initialpose", initial_pose_, &AmclNode::initialPoseReceived,this,2);
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
  map->origin_x = resp.map.origin.x + (map->size_x / 2) * map->scale;
  map->origin_y = resp.map.origin.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
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
  delete tfb_;
  delete tf_;
  pf_free(pf_);
  delete laser_;
  delete odom_;
  // TODO: delete everything allocated in constructor
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


pf_vector_t 
AmclNode::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

  for(;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;

    // Check that it's a free cell
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }

  return p;
}

bool
AmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  pf_mutex_.lock();
  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator, 
                (void *)map_);
  pf_init_ = false;
  pf_mutex_.unlock();
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
  pf_vector_t delta = pf_vector_zero();

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
    
    resample_count_ = 0;
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

  bool resampled = false;
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
    if(!(++resample_count_ % resample_interval_))
    {
      pf_update_resample(pf_);
      resampled = true;
    }

    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    ROS_INFO("Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud
    // TODO: set maximum rate for publishing
    robot_msgs::ParticleCloud cloud_msg;
    cloud_msg.set_particles_size(set->sample_count);
    for(int i=0;i<set->sample_count;i++)
    {
      tf::PoseTFToMsg(tf::Pose(btQuaternion(set->samples[i].pose.v[2], 0, 0), 
                               btVector3(set->samples[i].pose.v[0], 
                                         set->samples[i].pose.v[1], 0)),
                      cloud_msg.particles[i]);

    }
    ros::Node::instance()->publish("particlecloud", cloud_msg);
  }

  if(resampled)
  {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for(int hyp_count = 0; 
        hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if(hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if(max_weight > 0.0)
    {
      ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
                hyps[max_weight_hyp].pf_pose_mean.v[0],
                hyps[max_weight_hyp].pf_pose_mean.v[1],
                hyps[max_weight_hyp].pf_pose_mean.v[2]);

      /*
         puts("");
         pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
         puts("");
       */

      robot_msgs::PoseWithCovariance p;
      // Copy in the pose
      p.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::QuaternionTFToMsg(tf::Quaternion(hyps[max_weight_hyp].pf_pose_mean.v[2], 0.0, 0.0),
                            p.pose.orientation);
      // Copy in the covariance, converting from 3-D to 6-D
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
        }
      }
      p.covariance[6*3+3] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];

      /*
         printf("cov:\n");
         for(int i=0; i<6; i++)
         {
         for(int j=0; j<6; j++)
         printf("%6.3f ", p.covariance[6*i+j]);
         puts("");
         }
       */

      ros::Node::instance()->publish("amcl_pose", p);
      ROS_INFO("New pose: %6.3f %6.3f %6.3f %6.3f",
               p.pose.position.x,
               p.pose.position.y,
               p.pose.orientation.z,
               p.pose.orientation.w);

      // subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
        tf::Transform tmp_tf(tf::Quaternion(hyps[max_weight_hyp].pf_pose_mean.v[2], 
                                            0, 0), 
                             tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                                         hyps[max_weight_hyp].pf_pose_mean.v[1],
                                         0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(), 
                                              laser_scan->header.stamp, 
                                              "base_footprint");
        this->tf_->transformPose(odom_frame_id_, 
                                 tmp_tf_stamped,
                                 odom_to_map);
      }
      catch(tf::TransformException)
      {
        ROS_DEBUG("Failed to subtract base to odom transform");
        pf_mutex_.unlock();
        return;
      }

      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;

      // We want to send a transform that is good up until a 
      // tolerance time so that odom can be used
      ros::Time transform_expiration = (laser_scan->header.stamp +
                                        transform_tolerance_);
      tf::Stamped<tf::Transform> tmp_tf_stamped(latest_tf_.inverse(),
                                                transform_expiration,
                                                odom_frame_id_, "map");
      this->tfb_->sendTransform(tmp_tf_stamped);
    }
    else
    {
      ROS_ERROR("No pose!");
    }
  }
  else if(latest_tf_valid_)
  {
    ros::Time transform_expiration = (laser_scan->header.stamp +
                                      transform_tolerance_);
    tf::Stamped<tf::Transform> tmp_tf_stamped(latest_tf_.inverse(),
                                              transform_expiration,
                                              odom_frame_id_, "map");
    this->tfb_->sendTransform(tmp_tf_stamped);
  }

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
           initial_pose_.pose.position.x,
           initial_pose_.pose.position.y,
           getYaw(initial_pose_.pose));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = initial_pose_.pose.position.x;
  pf_init_pose_mean.v[1] = initial_pose_.pose.position.y;
  pf_init_pose_mean.v[2] = getYaw(initial_pose_.pose);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      pf_init_pose_cov.m[i][j] = initial_pose_.covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = initial_pose_.covariance[6*3+3];

  pf_mutex_.lock();
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;
  pf_mutex_.unlock();
}
