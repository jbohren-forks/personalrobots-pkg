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

#ifndef RDF_PARSER_JOINT_H
#define RDF_PARSER_JOINT_H

#include <string>
#include <vector>
#include <tinyxml/tinyxml.h>

#include <rdf_parser/pose.h>

using namespace std;

namespace rdf_parser{

class Joint
{
public:
  Joint(
    const std::string &name,
    TiXmlElement* xml_ = NULL,
    type = UNKNOWN,
    axis[0] = axis[1] = axis[2] = 0,
    anchor[0] = anchor[1] = anchor[2] = 0
    );

  TiXmlElement* xml_;
  std::string name_;

  enum
  {
    UNKNOWN, REVOLUTE, PRISMATIC, FLOATING, PLANAR, FIXED
  } type;

  /// \brief     TYPE        AXIS
  /// ------------------------------------------------------
  ///            REVOLUTE    rotation axis
  ///            PRISMATIC   translation axis
  ///            FLOATING    N/A
  ///            PLANAR      plane normal axis
  ///            FIXED       N/A
  Vector3 axis_;

  Link* parent_link_;
  Pose  parent_pose_;

  Link* child_link_;
  Pose  child_pose_;

  class JointProperties
  {
    JointProperties(
      damping = 0,
      friction = 0
      );
    double damping;
    double friction;
    virtual ~JointProperties(void) {};
  } joint_properties;

  class JointLimit
  {
  public:
    JointLimit(
      min = 0,
      max = 0
      effort = 0,
      velocity = 0,
      );
    double min;
    double max;
    double effort;
    double velocity;

    virtual ~JointLimit(void) {};

  } joint_limit;

  /// \brief A list of maps
  std::vector<TiXmlElement*> maps_;


  
private:

};

}

#endif
