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

#ifndef RobotModel_PARSER_JOINT_H
#define RobotModel_PARSER_JOINT_H

#include <string>
#include <vector>
#include <tinyxml/tinyxml.h>
#include <boost/shared_ptr.hpp>

#include <robot_model/pose.h>

namespace robot_model{

class Link;

class JointProperties
{
public:
  JointProperties() { this->clear(); };
  double damping_;
  double friction_;
protected:
  void clear()
  {
    damping_ = 0;
    friction_ = 0;
  };
  bool initXml(TiXmlElement* config);

  friend class Joint;
};

class JointLimits
{
public:
  JointLimits() { this->clear(); };
  double lower_;
  double upper_;
  double effort_;
  double velocity_;
protected:
  void clear()
  {
    lower_ = 0;
    upper_ = 0;
    effort_ = 0;
    velocity_ = 0;
  };
  bool initXml(TiXmlElement* config);

  friend class Joint;
};

class JointSafety
{
public:
  JointSafety() { this->clear(); };
  double soft_upper_limit_;
  double soft_lower_limit_;
  double k_p_;
  double k_v_;
protected:
  void clear()
  {
    soft_upper_limit_ = 0;
    soft_lower_limit_ = 0;
    k_p_ = 0;
    k_v_ = 0;
  };
  bool initXml(TiXmlElement* config);

  friend class Joint;
};

class JointCalibration
{
public:
  JointCalibration() { this->clear(); };
  double reference_position_;
protected:
  void clear()
  {
    reference_position_ = 0;
  };
  bool initXml(TiXmlElement* config);

  friend class Joint;
};

class Joint
{
public:

  Joint() { this->clear(); };

  std::string name_;
  enum
  {
    UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
  } type_;

  /// \brief     type_       meaning of axis_
  /// ------------------------------------------------------
  ///            UNKNOWN     unknown type
  ///            REVOLUTE    rotation axis
  ///            PRISMATIC   translation axis
  ///            FLOATING    N/A
  ///            PLANAR      plane normal axis
  ///            FIXED       N/A
  boost::shared_ptr<Vector3> axis_;

  /// child Link element
  boost::shared_ptr<Link> link_;
  std::string link_name_;
  ///   transform from Link frame to Joint frame
  boost::shared_ptr<Pose>  origin_;

  /// parent Link element
  boost::shared_ptr<Link> parent_link_;
  std::string parent_link_name_;
  ///   transform from parent Link to Joint frame
  boost::shared_ptr<Pose>  parent_origin_;

  /// Joint Properties
  boost::shared_ptr<JointProperties> joint_properties_;

  /// Joint Limits
  boost::shared_ptr<JointLimits> joint_limits_;

  /// Unsupported Hidden Feature
  boost::shared_ptr<JointSafety> joint_safety_;

  /// Unsupported Hidden Feature
  boost::shared_ptr<JointCalibration> joint_calibration_;

  // Joint element has one parent and one child Link
  void setParentLink(boost::shared_ptr<Link> parent) {this->parent_link_ = parent;};
  void setParentPose(boost::shared_ptr<Pose> pose) {this->parent_origin_ = pose;};

protected:
  bool initXml(TiXmlElement* xml);

private:
  void clear()
  {
    this->name_.clear();
    this->parent_link_.reset();
    this->axis_.reset();
    this->origin_.reset();
    this->parent_link_name_.clear();
    this->parent_origin_.reset();
    this->link_.reset();
    this->joint_properties_.reset();
    this->joint_limits_.reset();
    type_ = UNKNOWN;
  };

  friend class Link;
};

}

#endif
