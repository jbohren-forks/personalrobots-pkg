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

#include <pr2_mechanism_model/joint.h>
#include <map>
#include <string>
#include <vector>
#include <cfloat>
//#include "urdf/parser.h"

using namespace std;
using namespace pr2_mechanism;

static const pair<int, int> types[] = {
  pair<int, int>(urdf::Joint::UNKNOWN, JOINT_NONE),
  pair<int, int>(urdf::Joint::REVOLUTE, JOINT_ROTARY),
  pair<int, int>(urdf::Joint::CONTINUOUS, JOINT_CONTINUOUS),
  pair<int, int>(urdf::Joint::PRISMATIC, JOINT_PRISMATIC),
  pair<int, int>(urdf::Joint::FIXED, JOINT_FIXED),
  pair<int, int>(urdf::Joint::PLANAR, JOINT_PLANAR),
};

static map<int, int> g_type_map(types, types + sizeof(types)/sizeof(types[0]));



void Joint::enforceLimits(JointState *s)
{
  double vel_high = velocity_limit_;
  double vel_low = -velocity_limit_;
  double effort_high = effort_limit_;
  double effort_low = -effort_limit_;

  // only enforce joints that specify joint limits and safety code
  if (!has_safety_code_ || !has_joint_limits_)
    return;

  // enforce position bounds on rotary and prismatic joints that are calibrated
  if (s->calibrated_ && (type_ == JOINT_ROTARY || type_ == JOINT_PRISMATIC))
  {
    // Computes the velocity bounds based on the absolute limit and the
    // proximity to the joint limit.
    vel_high = max(-velocity_limit_,
                   min(velocity_limit_,
                       -k_position_limit_ * (s->position_ - safety_limit_max_)));
    vel_low = min(velocity_limit_,
                  max(-velocity_limit_,
                      -k_position_limit_ * (s->position_ - safety_limit_min_)));
  }

  // Computes the effort bounds based on the optional velocity bounds.
  if (velocity_limit_ >= 0.0)
  {
    effort_high = max(-effort_limit_,
                      min(effort_limit_,
                          -k_velocity_limit_ * (s->velocity_ - vel_high)));
    effort_low = min(effort_limit_,
                     max(-effort_limit_,
                         -k_velocity_limit_ * (s->velocity_ - vel_low)));
  }


  // limit the commanded effort based on position, velocity and effort limits
  s->commanded_effort_ =
    min( max(s->commanded_effort_, effort_low), effort_high);
}





bool Joint::init(const boost::shared_ptr<urdf::Joint> jnt)
{
  if (!jnt) return false;

  name_ = jnt->name;
  type_ = g_type_map[jnt->type];

  // get the optional safety parameters
  has_safety_code_ = false;
  if (jnt->safety){
    k_position_limit_ = jnt->safety->k_position;
    k_velocity_limit_ = jnt->safety->k_velocity;
    safety_limit_min_ = jnt->safety->soft_lower_limit;
    safety_limit_max_ = jnt->safety->soft_upper_limit;
    has_safety_code_ = true;
  }

  // get the optional joint limits 
  has_joint_limits_ = false;
  if (jnt->limits){
    joint_limit_min_ = jnt->limits->lower;
    joint_limit_max_ = jnt->limits->upper;
    effort_limit_ = jnt->limits->effort;
    velocity_limit_ = jnt->limits->velocity;
    has_joint_limits_ = true;
  }

  // get the optional calibration reference position
  if (jnt->calibration){
    reference_position_ = jnt->calibration->reference_position;
  }

  // get the optional joint properties
  if (jnt->dynamics){
    joint_damping_coefficient_ = jnt->dynamics->damping;
    joint_friction_coefficient_ = jnt->dynamics->friction;
  }
  
  if (has_safety_code_ && has_joint_limits_)
    ROS_DEBUG("Joint %s of type %i has safety code and limits: %f %f %f %f %f %f %f %f %f",  
              name_.c_str(), type_, safety_limit_min_, safety_limit_max_,
              k_position_limit_, k_velocity_limit_, joint_limit_min_, joint_limit_max_, 
              effort_limit_, velocity_limit_, joint_damping_coefficient_);
  else 
    ROS_DEBUG("Joint %s has not safety code or limits", name_.c_str()); 
  

  return true;
}

