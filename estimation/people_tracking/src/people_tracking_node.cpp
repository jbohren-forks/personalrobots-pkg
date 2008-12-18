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
#include "state_pos_vel.h"

using namespace std;
using namespace ros;
using namespace tf;
using namespace BFL;

static const unsigned int num_trackers = 1;


namespace estimation
{
  // constructor
  PeopleTrackingNode::PeopleTrackingNode(const string& node_name)
    : ros::node(node_name),
      node_name_(node_name),
      time_(0),
      move_(Vector3(0,0,0), Vector3(0.5,0.5,0.000000001))
  {
    param(node_name_+"/freq", freq_, 1.0);
    Vector3 prior_mu_pos;
    param(node_name_+"/prior/x", prior_mu_pos[0] , 0.0);
    param(node_name_+"/prior/y", prior_mu_pos[1] , 0.0);
    param(node_name_+"/prior/z", prior_mu_pos[2] , 0.0);

    // create sample trackers
    StatePosVel prior_mu(prior_mu_pos, Vector3(0, 0, 0));
    StatePosVel prior_sigma(Vector3(0.3, 0.3, 0.00001), Vector3(0.00001, 0.00001, 0.00001));
    StatePosVel sys_sigma(Vector3(0.1, 0.1, 0.00001), Vector3(0.1, 0.1, 0.00001));
    Vector3 meas_sigma(0.3, 0.3, 1000);

    Tracker* tr;
    for (unsigned int i=0; i<num_trackers; i++){
      // tracker
      tr = new Tracker(5000, sys_sigma, meas_sigma);
      tr->initialize(prior_mu, prior_sigma, time_);
      trackers_.push_back(tr);

      // move / vel
      meas_.push_back(prior_mu.pos_);
      vel_.push_back(Vector3(0,0,0));
    }

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
	cout << " - quality        = " << trackers_[i]->getQuality() << endl;
	// publish result
	trackers_[i]->getParticleCloud(Vector3(0.06, 0.06, 0.06), 0.0001, cloud);
	publish("people_tracking", cloud);
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
