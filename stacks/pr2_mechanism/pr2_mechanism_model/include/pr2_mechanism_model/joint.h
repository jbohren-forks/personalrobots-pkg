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
/*
 *
 */
#ifndef JOINT_H
#define JOINT_H

#include <tinyxml/tinyxml.h>
#include <urdf/joint.h>

namespace pr2_mechanism {

class JointState;

class Joint{
public:
  Joint() :
    type_(0),
    joint_limit_min_(0.0), joint_limit_max_(0.0),
    effort_limit_(0.0), velocity_limit_(0.0),
    joint_damping_coefficient_(0.0), joint_friction_coefficient_(0.0),
    k_position_limit_(0.0), k_velocity_limit_(0.0),
    safety_limit_min_(0.0),safety_limit_max_(0.0),
    reference_position_(0.0) {}
  ~Joint() {}

  void enforceLimits(JointState *s);
  bool init(const boost::shared_ptr<urdf::Joint> jnt);

  std::string name_;
  int type_;

  // Joint limits
  double joint_limit_min_;  // In radians
  double joint_limit_max_;  // In radians
  double effort_limit_;
  double velocity_limit_;
  bool has_joint_limits_;

  // Joint Properites
  double joint_damping_coefficient_; // non-dimensional
  double joint_friction_coefficient_;

  // safety controller parameters
  double k_position_limit_;
  double k_velocity_limit_;
  double safety_limit_min_;
  double safety_limit_max_;
  bool has_safety_code_;

  // calibration parameters
  double reference_position_;
};


class JointState
{
public:
  Joint *joint_;

  // State
  double position_;  // In radians
  double velocity_;
  double applied_effort_;

  // Command
  double commanded_effort_;

  bool calibrated_;

  JointState() : joint_(NULL), position_(0.0), velocity_(0.0), applied_effort_(0.0),
                 commanded_effort_(0), calibrated_(false) {}
  JointState(const JointState &s)
    : joint_(s.joint_), position_(s.position_), velocity_(s.velocity_),
      applied_effort_(s.applied_effort_), commanded_effort_(s.commanded_effort_), calibrated_(s.calibrated_)
  {}
};

enum
{
  JOINT_NONE,
  JOINT_ROTARY,
  JOINT_CONTINUOUS,
  JOINT_PRISMATIC,
  JOINT_FIXED,
  JOINT_PLANAR,
  JOINT_TYPES_MAX
};

}

#endif /* JOINT_H */
