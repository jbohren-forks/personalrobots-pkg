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

/* Author: Wim Meeussen */

#include "people_tracking_node.h"
#include "tracker_particle.h"
#include "tracker_kalman.h"
#include "state_pos_vel.h"
#include "rgb.h"
#include <robot_msgs/PositionMeasurement.h>


using namespace std;
using namespace ros;
using namespace tf;
using namespace BFL;
using namespace robot_msgs;


static const unsigned int num_meas_show              = 10;
static const double       sequencer_delay            = 0.5;
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker      = 1000;


namespace estimation
{
  // constructor
  PeopleTrackingNode::PeopleTrackingNode(const string& node_name)
    : ros::node(node_name),
      node_name_(node_name),
      message_sequencer_(this, "people_tracker_measurements", 
			 boost::bind(&PeopleTrackingNode::callbackRcv,  this, _1),
			 boost::bind(&PeopleTrackingNode::callbackDrop, this, _1),
			 ros::Duration().fromSec(sequencer_delay), 
			 sequencer_internal_buffer, sequencer_subscribe_buffer),
      robot_state_(*this, true),
      tracker_counter_(0),
      meas_visualize_(num_meas_show),
      meas_visualize_counter_(0)
  {
    // initialize
    for (unsigned int i=0; i<num_meas_show; i++){
      meas_visualize_[i].x = 0;
      meas_visualize_[i].y = 0;
      meas_visualize_[i].z = 0;
    }
    // get parameters
    param("~/fixed_frame", fixed_frame_, string("default"));
    param("~/freq", freq_, 1.0);
    param("~/sys_sigma_pos_x", sys_sigma_.pos_[0], 0.0);
    param("~/sys_sigma_pos_y", sys_sigma_.pos_[1], 0.0);
    param("~/sys_sigma_pos_z", sys_sigma_.pos_[2], 0.0);
    param("~/sys_sigma_vel_x", sys_sigma_.vel_[0], 0.0);
    param("~/sys_sigma_vel_y", sys_sigma_.vel_[1], 0.0);
    param("~/sys_sigma_vel_z", sys_sigma_.vel_[2], 0.0);
    sys_sigma_.pos_ /= freq_;
    sys_sigma_.vel_ /= freq_;

    // advertise filter output
    advertise<robot_msgs::PositionMeasurement>("people_tracker_filter",10);

    // advertise visualization
    advertise<std_msgs::PointCloud>("people_tracker_filter_visualization",10);
    advertise<std_msgs::PointCloud>("people_tracker_measurements_visualization",10);
  }




  // destructor
  PeopleTrackingNode::~PeopleTrackingNode()
  {
    // delete all trackers
    for (map<string, Tracker*>::iterator it= trackers_.begin(); it!=trackers_.end(); it++)
      delete it->second;
  };




  // callback for messages
  void PeopleTrackingNode::callbackRcv(const boost::shared_ptr<robot_msgs::PositionMeasurement>& message)
  {
    // get time and object name
    double time(message->header.stamp.toSec());
    string name(message->object_id);

    // get measurement in fixed frame
    Stamped<Vector3> meas_rel, meas;
    meas_rel.setData(StateVector(message->pos.x, message->pos.y, message->pos.z));
    meas_rel.stamp_    = message->header.stamp;
    meas_rel.frame_id_ = message->header.frame_id;
    robot_state_.transformPoint(fixed_frame_, meas_rel, meas);

    // get measurement covariance
    SymmetricMatrix cov(3);
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
	cov(i+1, j+1) = message->covariance[3*i+j];

    // ----- LOCKED ------
    boost::mutex::scoped_lock lock(filter_mutex_);

    // update tracker if matching tracker found
    tracker_it_ = trackers_.find(name);
    if (tracker_it_ != trackers_.end()){
      ROS_INFO("%s: Update tracker %s with measurement from %s",  
	       node_name_.c_str(), name.c_str(), message->name.c_str());
      tracker_it_->second->updateCorrection(meas, cov, time);
    }

    // initialize a new tracker when initialization flag is set
    if (message->initialization == 1){
      stringstream name_new;
      // TODO: WHAT SHOULD COVAR OF VEL BE??
      StatePosVel prior_sigma(StateVector(sqrt(cov(1,1)), sqrt(cov(2,2)),sqrt(cov(3,3))), StateVector(0.0000001, 0.0000001, 0.0000001));
      name_new << "person " << tracker_counter_++;
      trackers_[name_new.str()] = new TrackerKalman(sys_sigma_);
      //trackers_[name_new.str()] = new TrackerParticle(num_particles_tracker, sys_sigma_);
      trackers_[name_new.str()]->initialize(meas, prior_sigma, time);
      ROS_INFO("%s: Initialized new tracker %s", node_name_.c_str(), name_new.str().c_str());
    }
    lock.unlock();
    // ------ LOCKED ------

    
    // visualize measurement
    meas_visualize_[meas_visualize_counter_].x = meas[0];
    meas_visualize_[meas_visualize_counter_].y = meas[1];
    meas_visualize_[meas_visualize_counter_].z = meas[2];
    meas_visualize_counter_++;
    if (meas_visualize_counter_ == num_meas_show) meas_visualize_counter_ = 0;
    std_msgs::PointCloud  meas_cloud; 
    meas_cloud.header.frame_id = meas.frame_id_;
    meas_cloud.pts = meas_visualize_;
    publish("people_tracker_measurements_visualization", meas_cloud);

  }



  // callback for dropped messages
  void PeopleTrackingNode::callbackDrop(const boost::shared_ptr<robot_msgs::PositionMeasurement>& message)
  {
    ROS_INFO("%s: DROPPED PACKAGE for %s from %s with delay %f !!!!!!!!!!!", 
	     node_name_.c_str(), message->object_id.c_str(), message->name.c_str(), (ros::Time::now() - message->header.stamp).toSec());

  }




  // filter loop
  void PeopleTrackingNode::spin()
  {
    ROS_INFO("%s: People tracking manager stated.", node_name_.c_str());

    while (ok()){
      // visualization variables
      vector<std_msgs::Point32> filter_visualize(trackers_.size());
      vector<float> weights(trackers_.size());
      std_msgs::ChannelFloat32 channel;


      // ------ LOCKED ------
      // loop over trackers
      boost::mutex::scoped_lock lock(filter_mutex_);
      unsigned int i=0;
      for (map<string, Tracker*>::iterator it= trackers_.begin(); it!=trackers_.end(); it++,i++){

	// update prediction
	it->second->updatePrediction(1/freq_);

	// publish filter result
	PositionMeasurement est_pos;
	it->second->getEstimate(est_pos);
	est_pos.object_id = it->first;
	est_pos.header.frame_id = fixed_frame_;
	publish("people_tracker_filter", est_pos);

	filter_visualize[i].x = est_pos.pos.x;
	filter_visualize[i].y = est_pos.pos.y;
	filter_visualize[i].z = est_pos.pos.z;
	weights[i] = rgb[min(998, 999-max(1, (int)trunc( it->second->getQuality()*999.0 )))];

	// remove trackers that have zero quality
	ROS_INFO("%s: quality of tracker %s = %f",node_name_.c_str(), it->first.c_str(), it->second->getQuality());
	if (it->second->getQuality() <= 0){
	  ROS_INFO("%s: Removing tracker %s",node_name_.c_str(), it->first.c_str());  
	  trackers_.erase(it);
	}
      }
      lock.unlock();
      // ------ LOCKED ------


      // visualize all trackers
      channel.name = "rgb";
      channel.vals = weights;
      std_msgs::PointCloud  people_cloud; 
      people_cloud.chan.push_back(channel);
      people_cloud.header.frame_id = fixed_frame_;
      people_cloud.pts  = filter_visualize;
      publish("people_tracker_filter_visualization", people_cloud);

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
  string node_name("people_tracker");

  // create tracker node
  PeopleTrackingNode my_tracking_node(node_name);

  // wait for filter to finish
  my_tracking_node.spin();

  // Clean up
  ros::fini();
  return 0;
}
