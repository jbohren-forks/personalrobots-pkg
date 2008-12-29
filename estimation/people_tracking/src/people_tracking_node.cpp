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

using namespace std;
using namespace ros;
using namespace tf;
using namespace BFL;

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
      message_sequencer_(this, "topicname", 
			 boost::bind(&PeopleTrackingNode::callbackRcv, this, _1),
			 boost::bind(&PeopleTrackingNode::callbackDrop, this, _1),
			 ros::Duration().fromSec(sequencer_delay), 
			 sequencer_internal_buffer, sequencer_subscribe_buffer),
      time_(0),
      move_(Vector3(0,0,0), Vector3(0.5,0.5,0.000000001)),
      sys_sigma_(Vector3(0.1, 0.1, 0.00001), Vector3(0.1, 0.1, 0.00001)),
      meas_sigma_(0.3, 0.3, 1000),
      prior_sigma_(Vector3(0.3, 0.3, 0.00001), Vector3(0.00001, 0.00001, 0.00001))
  {
    // get parameters
    param(node_name_+"/freq", freq_, 1.0);

    // advertise
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
	   message->name.c_str(), 
	   message->header.stamp.toSec(), 
	   message->pos.x, 
	   message->pos.y, 
	   message->pos.z);


    // get stuff from message
    double time(message->header.stamp.toSec());
    Vector3 pos(message->pos.x, message->pos.y, message->pos.z);
    string name(message->name);

    // search for tracker name in tracker list
    Tracker* tracker = NULL;
    for (unsigned int i=0; i<tracker_names_.size(); i++)
      if (name == tracker_names_[i])
	tracker = trackers_[i];


    // update tracker
    if (tracker != NULL){
      tracker->updatePrediction(time);
      tracker->updateCorrection(pos);
    }


    // initialize a new tracker
    if (message->initialization == 1){
      ROS_INFO("%s: Initializing a new tracker: %s", node_name_.c_str(), name.c_str());
      Tracker* tracker_new = new TrackerKalman(sys_sigma_, meas_sigma_);
      tracker_new->initialize(pos, prior_sigma_, time);
      trackers_.push_back(tracker_new);
      tracker_names_.push_back(name);
    }
  }



  // callback for dropped messages
  void PeopleTrackingNode::callbackDrop(const boost::shared_ptr<robot_msgs::PositionMeasurement>& message)
  {
    printf("Dropped a position measurement from %s at %f with: %f %f %f\n", 
	   message->name.c_str(), 
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
      time_ += 1/freq_;
      std_msgs::PointCloud cloud;

      for (unsigned int i=0; i<num_trackers; i++){
	// update trackers
	move_.SampleFrom(sample);
	vel_[i] += sample.ValueGet();
	meas_[i] += (vel_[i] / freq_);
	trackers_[i]->updatePrediction(time_);
	trackers_[i]->updateCorrection(meas_[i]);
	cout << "Tracker " << i << endl;
	cout << " - expected value = " << trackers_[i]->getEstimate() << endl;
	cout << " - measurement    = " << StatePosVel(meas_[i], Vector3(0,0,0)) << endl;
	cout << " - velocity       = " << StatePosVel(Vector3(0,0,0), vel_[i]) << endl;
	cout << " - quality        = " << trackers_[i]->getQuality() << endl;
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
