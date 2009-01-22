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
#ifndef MECHANISM_LINK_H
#define MECHANISM_LINK_H

#include "mechanism_model/joint.h"
#include <vector>
#include "libTF/Pose3D.h"
#include "tf/transform_datatypes.h"


namespace mechanism {

class Robot;

class Link
{
public:
  Link();
  ~Link() {}

  bool initXml(TiXmlElement *config, Robot *robot);

  std::string name_;
  std::string parent_name_;
  std::string joint_name_;

  tf::Vector3 origin_xyz_;
  tf::Vector3 origin_rpy_;

  tf::Transform getOffset()
  {
    return tf::Transform(tf::Quaternion(0,0,0), origin_xyz_);
  }

  tf::Transform getRotation()
  {
    return tf::Transform(tf::Quaternion(origin_rpy_[2], origin_rpy_[1], origin_rpy_[0]));
  }
};

class LinkState
{
public:
  Link *link_;

  tf::Transform rel_frame_;  // Relative transform to the parent link's frame.
  tf::Vector3 abs_position_;  // Absolute position (in the robot frame)
  tf::Quaternion abs_orientation_;  // Absolute orientation (in the robot frame)
  tf::Vector3 abs_velocity_;
  tf::Vector3 abs_rot_velocity_;

  void propagateFK(LinkState *parent, JointState *joint);

  LinkState() : link_(NULL) {}
  LinkState(const LinkState &s) : link_(s.link_), rel_frame_(s.rel_frame_) {}
};


} // namespace mechanism

#endif
