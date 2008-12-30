/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "people_tracking_node.h"
#include "tracker_particle.h"
#include "tracker_kalman.h"
#include "state_pos_vel.h"
#include <robot_msgs/PositionMeasurement.h>


using namespace std;
using namespace ros;
using namespace tf;
using namespace BFL;
using namespace robot_msgs;


static const unsigned int num_meas_show              = 10;
static const double       sequencer_delay            = 0.2;
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;


namespace estimation
{
  // constructor
  PeopleTrackingNode::PeopleTrackingNode(const string& node_name)
    : ros::node(node_name),
      node_name_(node_name),
      message_sequencer_(this, "people_tracking_measurements", 
			 boost::bind(&PeopleTrackingNode::callbackRcv,  this, _1),
			 boost::bind(&PeopleTrackingNode::callbackDrop, this, _1),
			 ros::Duration().fromSec(sequencer_delay), 
			 sequencer_internal_buffer, sequencer_subscribe_buffer),
      robot_state_(*this, true),
      tracker_counter_(0),
      meas_vis_(num_meas_show),
      meas_vis_counter_(0)
  {
    // initialize
    for (unsigned int i=0; i<num_meas_show; i++){
      meas_vis_[i].x = 0;
      meas_vis_[i].y = 0;
      meas_vis_[i].z = 0;
    }

    // get parameters
    param("~/freq", freq_, 1.0);

    param("~/sys_sigma_pos_x", sys_sigma_.pos_[0], 0.0);
    param("~/sys_sigma_pos_y", sys_sigma_.pos_[1], 0.0);
    param("~/sys_sigma_pos_z", sys_sigma_.pos_[2], 0.0);
    param("~/sys_sigma_vel_x", sys_sigma_.vel_[0], 0.0);
    param("~/sys_sigma_vel_y", sys_sigma_.vel_[1], 0.0);
    param("~/sys_sigma_vel_z", sys_sigma_.vel_[2], 0.0);

    param("~/prior_sigma_pos_x", prior_sigma_.pos_[0], 0.0);
    param("~/prior_sigma_pos_y", prior_sigma_.pos_[1], 0.0);
    param("~/prior_sigma_pos_z", prior_sigma_.pos_[2], 0.0);
    param("~/prior_sigma_vel_x", prior_sigma_.vel_[0], 0.0);
    param("~/prior_sigma_vel_y", prior_sigma_.vel_[1], 0.0);
    param("~/prior_sigma_vel_z", prior_sigma_.vel_[2], 0.0);

    // advertise
    advertise<robot_msgs::PositionMeasurement>("people_tracking_filter",10);

    // advertise visualization
    advertise<std_msgs::PointCloud>("people_tracking_filter_visualization",10);
    advertise<std_msgs::PointCloud>("people_tracking_meassurements_visualization",10);
  }




  // destructor
  PeopleTrackingNode::~PeopleTrackingNode()
  {
    // delete all trackers
    for (unsigned int i=0; i<trackers_.size(); i++)
      delete trackers_[i];
  };




  // callback for messages
  void PeopleTrackingNode::callbackRcv(const boost::shared_ptr<robot_msgs::PositionMeasurement>& message)
  {
    // get time and object name
    double time(message->header.stamp.toSec());
    string name(message->object_id);

    // get position
    Stamped<Vector3> pos_rel, pos_abs;
    pos_rel.setData(Vector3(message->pos.x, message->pos.y, message->pos.z));
    pos_rel.stamp_    = message->header.stamp;
    pos_rel.frame_id_ = message->header.frame_id;
    // TODO: REMOVE AFTER DEBUGGING
    //robot_state_.transformVector("odom_combined", pos_rel, pos_abs);
    pos_abs = pos_rel;

    // get position covariance
    SymmetricMatrix cov(3);
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
	cov(i+1, j+1) = message->covariance[3*i+j];

    // search for tracker name in tracker list
    Tracker* tracker = NULL;
    for (unsigned int i=0; i<tracker_names_.size(); i++)
      if (name == tracker_names_[i])
	tracker = trackers_[i];

    // update tracker if matching tracker found
    if (tracker != NULL){
      ROS_INFO("%s: Update tracker %s with measurement from %s",  
	       node_name_.c_str(), name.c_str(), message->name.c_str());
      tracker->updatePrediction(time);
      tracker->updateCorrection(pos_abs, cov);
    }

    // initialize a new tracker
    if (message->initialization == 1){
      stringstream name_new;
      name_new << "person " << tracker_counter_++;
      Tracker* tracker_new = new TrackerKalman(sys_sigma_, meas_sigma_);
      tracker_new->initialize(pos_abs, prior_sigma_, time);
      trackers_.push_back(tracker_new);
      tracker_names_.push_back(name_new.str());

      ROS_INFO("%s: Initialized new tracker %s", node_name_.c_str(), name_new.str().c_str());
    }

    // visualize measurement
    meas_vis_[meas_vis_counter_].x = pos_abs[0];
    meas_vis_[meas_vis_counter_].y = pos_abs[1];
    meas_vis_[meas_vis_counter_].z = pos_abs[2];
    meas_vis_counter_++;
    if (meas_vis_counter_ == num_meas_show) meas_vis_counter_ = 0;

    std_msgs::PointCloud  meas_cloud; 
    meas_cloud.header.frame_id = "odom";
    meas_cloud.pts = meas_vis_;
    publish("people_tracking_meassurements_visualization", meas_cloud);

  }



  // callback for dropped messages
  void PeopleTrackingNode::callbackDrop(const boost::shared_ptr<robot_msgs::PositionMeasurement>& message)
  {
    ROS_INFO("%s: DROPPED PACKAGE FROM %s !!!!!!!!!!!", node_name_.c_str(), message->object_id.c_str());
  }




  // filter loop
  void PeopleTrackingNode::spin()
  {
    ROS_INFO("%s: People tracking manager stated.", node_name_.c_str());

    while (ok()){

      // publish result
      vector<std_msgs::Point32> points(trackers_.size());
      for (unsigned int i=0; i<trackers_.size(); i++){
	PositionMeasurement est_pos;
	trackers_[i]->getEstimate(est_pos);
	est_pos.object_id = tracker_names_[i];
	points[i].x = est_pos.pos.x;
	points[i].y = est_pos.pos.y;
	points[i].z = est_pos.pos.z;

	publish("people_tracking_filter", est_pos);
      }
      std_msgs::PointCloud  people_cloud; 
      people_cloud.header.frame_id = "odom";
      people_cloud.pts  = points;
      publish("people_tracking_filter_visualization", people_cloud);

      // sleep
      usleep(1e6/freq_);
    }
  };


}; // namespace






// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv);

  // get node name from arguments
  string node_name("people_tracking");

  // create tracker node
  PeopleTrackingNode my_tracking_node(node_name);

  // wait for filter to finish
  my_tracking_node.spin();

  // Clean up
  ros::fini();
  return 0;
}
