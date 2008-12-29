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


static const unsigned int num_trackers               = 2;
static const double sequencer_delay                  = 0.2;
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
      robot_state_(*this, true)
  {
    // get parameters
    param("~/freq", freq_, 1.0);

    param("~/sys_sigma_pos_x", sys_sigma_.pos_[0], 0.0);
    param("~/sys_sigma_pos_y", sys_sigma_.pos_[1], 0.0);
    param("~/sys_sigma_pos_z", sys_sigma_.pos_[2], 0.0);
    param("~/sys_sigma_vel_x", sys_sigma_.vel_[0], 0.0);
    param("~/sys_sigma_vel_y", sys_sigma_.vel_[1], 0.0);
    param("~/sys_sigma_vel_z", sys_sigma_.vel_[2], 0.0);

    param("~/meas_sigma_x", meas_sigma_[0], 0.0);
    param("~/meas_sigma_y", meas_sigma_[1], 0.0);
    param("~/meas_sigma_z", meas_sigma_[2], 0.0);

    param("~/prior_sigma_pos_x", prior_sigma_.pos_[0], 0.0);
    param("~/prior_sigma_pos_y", prior_sigma_.pos_[1], 0.0);
    param("~/prior_sigma_pos_z", prior_sigma_.pos_[2], 0.0);
    param("~/prior_sigma_vel_x", prior_sigma_.vel_[0], 0.0);
    param("~/prior_sigma_vel_y", prior_sigma_.vel_[1], 0.0);
    param("~/prior_sigma_vel_z", prior_sigma_.vel_[2], 0.0);

    // advertise
    advertise<robot_msgs::PositionMeasurement>("people_tracking_filter",10);
    advertise<std_msgs::PointCloud>("people_tracking",3);
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
    printf("Got a position measurement from %s at %f with: %f %f %f\n", 
	   message->object_id.c_str(), 
	   message->header.stamp.toSec(), 
	   message->pos.x, 
	   message->pos.y, 
	   message->pos.z);


    // get stuff from message
    double time(message->header.stamp.toSec());
    string name(message->object_id);
    Stamped<Vector3> pos_rel, pos_abs;
    pos_rel.setData(Vector3(message->pos.x, message->pos.y, message->pos.z));
    pos_rel.stamp_    = message->header.stamp;
    pos_rel.frame_id_ = message->header.frame_id;
    robot_state_.transformVector("odom_combined", pos_rel, pos_abs);
    // TODO: REMOVE AFTER DEBUGGING
    pos_abs = pos_rel;

    // search for tracker name in tracker list
    Tracker* tracker = NULL;
    for (unsigned int i=0; i<tracker_names_.size(); i++)
      if (name == tracker_names_[i])
	tracker = trackers_[i];

    // update tracker
    if (tracker != NULL){
      tracker->updatePrediction(time);
      tracker->updateCorrection(pos_abs);
    }

    // initialize a new tracker
    if (message->initialization == 1){
      ROS_INFO("%s: Initializing a new tracker: %s", node_name_.c_str(), name.c_str());
      Tracker* tracker_new = new TrackerKalman(sys_sigma_, meas_sigma_);
      tracker_new->initialize(pos_abs, prior_sigma_, time);
      trackers_.push_back(tracker_new);
      tracker_names_.push_back(name);
    }
  }



  // callback for dropped messages
  void PeopleTrackingNode::callbackDrop(const boost::shared_ptr<robot_msgs::PositionMeasurement>& message)
  {
    printf("Dropped a position measurement from %s at %f with: %f %f %f\n", 
	   message->object_id.c_str(), 
	   message->header.stamp.toSec(), 
	   message->pos.x, 
	   message->pos.y, 
	   message->pos.z);
  }



  // filter loop
  void PeopleTrackingNode::spin()
  {
    while (ok()){
      Sample<Vector3> sample;
      std_msgs::PointCloud cloud;

      for (unsigned int i=0; i<trackers_.size(); i++){
	StatePosVel est_vec;
	trackers_[i]->getEstimate(est_vec);
	cout << "Tracker " << i << " - expected value = " << est_vec << endl;

	PositionMeasurement est_pos;
	trackers_[i]->getEstimate(est_pos);
	publish("people_tracking_filter", est_pos);

	// publish result
	//((TrackerParticle*)(trackers_[i]))->getParticleCloud(Vector3(0.06, 0.06, 0.06), 0.0001, cloud);
	//publish("people_tracking", cloud);

      }
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
