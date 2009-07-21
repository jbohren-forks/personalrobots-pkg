/*
 * Copyright (C) 2008, Jason Wolfe and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
#include <map>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <planning_environment/monitors/planning_monitor.h>
#include <planning_models/kinematic.h>
#include <boost/foreach.hpp>
#include <boost/scoped_array.hpp>
#include <tf/tf.h>
#include <fk_node/ForwardKinematics.h>


#define foreach BOOST_FOREACH


namespace fk_node
{

using std::string;
using std::vector;
using std::copy;


namespace pe=planning_environment;
namespace pm=planning_models;
namespace mpm=motion_planning_msgs;

typedef boost::shared_ptr<pm::StateParams> State;
typedef boost::shared_ptr<pm::KinematicModel> KinModelPtr;
typedef pm::KinematicModel::Link* LinkPtr;
typedef boost::scoped_array<double> DoubleArray;

class ForwardKinematicsNode
{
public:

  ForwardKinematicsNode();
  bool computeForwardKinematics(ForwardKinematics::Request& req, ForwardKinematics::Response& resp);

private:

  ros::NodeHandle node_;
  tf::TransformListener tf_;
  pe::CollisionModels collision_models_;
  pe::PlanningMonitor monitor_;
  KinModelPtr kinematic_model_;
  ros::ServiceServer fk_service_;
};


unsigned getDimension(const KinModelPtr& model)
{
  State state(model->newStateParams());
  vector<double> params;
  state->copyParams(params);
  return params.size();
}



ForwardKinematicsNode::ForwardKinematicsNode () :
  collision_models_("robot_description"), monitor_(&collision_models_, &tf_),
  kinematic_model_(collision_models_.getKinematicModel())
{
  if (collision_models_.loadedModels()) {
    monitor_.getEnvironmentModel()->setVerbose(true);
    // monitor.waitForState();
    // monitor.waitForMap();
  }

  fk_service_ = node_.advertiseService("~forward_kinematics", &ForwardKinematicsNode::computeForwardKinematics, this);
  
}

  



bool ForwardKinematicsNode::computeForwardKinematics(ForwardKinematics::Request& req, ForwardKinematics::Response& resp)
{
  State state(kinematic_model_->newStateParams());
  state->defaultState();
  
  foreach (mpm::KinematicJoint joint, req.joints) {
    DoubleArray value_array(new double[joint.value.size()]);
    copy(joint.value.begin(), joint.value.end(), value_array.get());
    state->setParamsJoint(value_array.get(), joint.joint_name);
  }

  kinematic_model_->computeTransforms(state->getParams());
  vector<LinkPtr> links; 
  kinematic_model_->getLinks(links);
  foreach (LinkPtr link, links) {
    resp.link_names.push_back(link->name);
    robot_msgs::Pose pose;
    tf::poseTFToMsg(link->globalTrans, pose);
    resp.link_poses.push_back(pose);
  }

  return true;
}


} // namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fk_node");
  fk_node::ForwardKinematicsNode fk;
  ros::spin();
} 

