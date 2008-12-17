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


namespace estimation
{
  // constructor
  PeopleTrackingNode::PeopleTrackingNode(const string& node_name)
    : ros::node(node_name),
      node_name_(node_name),
      time_(0),
      vel_(0,0,0),
      move_(Vector3(0,0,0), Vector3(0.002,0.002,0.000000001))
  {
    param(node_name_+"/freq", freq_, 20.0);

    // create sample tracker
    StatePosVel prior_mu(Vector3(3, 3, 3), Vector3(0, 0, 0));
    StatePosVel prior_sigma(Vector3(0.3, 0.3, 0.00001), Vector3(0.00001, 0.00001, 0.00001));
    StatePosVel sys_sigma(Vector3(0.1, 0.1, 0.00001), Vector3(0.1, 0.1, 0.00001));
    Vector3 meas_sigma(0.3, 0.3, 1000);
    Tracker* tr = new Tracker(5000, sys_sigma, meas_sigma);
    tr->initialize(prior_mu, prior_sigma, time_);
    trackers_.push_back(tr);

    // advertise
    advertise<std_msgs::PointCloud>("people_tracking",3);

    // init
    meas_ = prior_mu.pos_;
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
      move_.SampleFrom(sample);
      vel_ += sample.ValueGet();
      meas_ += vel_;
      time_ += 1/freq_;
      trackers_[0]->updatePrediction(time_);
      trackers_[0]->updateCorrection(meas_);
      cout << "expected value = " << trackers_[0]->getEstimate() << endl;
      cout << "measurement    = " << StatePosVel(meas_, Vector3(0,0,0)) << endl;

      // publish result
      std_msgs::PointCloud cloud;
      trackers_[0]->getParticleCloud(Vector3(0.06, 0.06, 0.06), 0.0001, cloud);
      publish("people_tracking", cloud);

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
  string node_name("people_tracking_node");

  // create tracker node
  PeopleTrackingNode my_tracking_node(node_name);

  // wait for filter to finish
  my_tracking_node.spin();

  // Clean up
  ros::fini();
  return 0;
}
