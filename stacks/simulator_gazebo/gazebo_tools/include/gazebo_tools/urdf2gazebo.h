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
#ifndef URDF2GAZEBO_HH
#define URDF2GAZEBO_HH

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <vector>
#include <string>

#include <sstream>
#include <map>
#include <vector>

#include <urdf/model.h>

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"

namespace urdf2gazebo
{
  double rad2deg(double v) { return v * 180.0 / M_PI; };

  class GazeboExtension
  {
    public:
      // visual
      std::string material;

      // body, default off
      bool turnGravityOff;
      bool is_dampingFactor;
      double dampingFactor;
      bool selfCollide;

      // geom, contact dynamics
      bool is_mu1, is_mu2, is_kp, is_kd;
      double mu1, mu2, kp, kd;
      bool is_laserRetro;
      double laserRetro;

      // joint, joint limit dynamics
      bool is_stopKd, is_stopKp, is_fudgeFactor;
      double stopKp, stopKd, fudgeFactor;
      bool provideFeedback;

      // copy blocks into body or robot
      std::vector<TiXmlElement*> copy_block;

      GazeboExtension()
      {
        //initialize
        material.clear();
        turnGravityOff = false;
        is_dampingFactor = false;
        is_mu1 = false;
        is_mu2 = false;
        is_kp = false;
        is_kd = false;
        selfCollide = false;
        is_laserRetro = false;
        is_stopKp = false;
        is_stopKd = false;
        is_fudgeFactor = false;
        provideFeedback = false;
        copy_block.clear();
      };
  };


  class URDF2Gazebo
  {
    public:
      URDF2Gazebo();
      URDF2Gazebo(std::string robot_model_name);
      ~URDF2Gazebo();

      std::string values2str(unsigned int count, const double *values, double (*conv)(double));
      std::string vector32str(const urdf::Vector3 vector, double (*conv)(double));

      void setupTransform(btTransform &transform, urdf::Pose pose);

      void addKeyValue(TiXmlElement *elem, const std::string& key, const std::string &value);

      void addTransform(TiXmlElement *elem, const::btTransform& transform);

      std::string getGazeboValue(TiXmlElement* elem);
      void parseGazeboExtension(TiXmlDocument &urdf_in);
      void insertGazeboExtensionGeom(TiXmlElement *elem,std::string link_name);
      void insertGazeboExtensionVisual(TiXmlElement *elem,std::string link_name);
      void insertGazeboExtensionBody(TiXmlElement *elem,std::string link_name);
      void insertGazeboExtensionJoint(TiXmlElement *elem,std::string joint_name);
      void insertGazeboExtensionRobot(TiXmlElement *elem);

      std::string getGeometrySize(boost::shared_ptr<urdf::Geometry> geometry, int *sizeCount, double *sizeVals);
      
      std::string getGeometryBoundingBox(boost::shared_ptr<urdf::Geometry> geometry, double *sizeVals);

      void convertLink(TiXmlElement *root, boost::shared_ptr<const urdf::Link> link, const btTransform &transform, bool enforce_limits);

      void convert( TiXmlDocument &urdf_in, TiXmlDocument &gazebo_xml_out, bool enforce_limits, urdf::Vector3 initial_xyz, urdf::Vector3 initial_rpy,bool xml_declaration = false);

      std::string robot_model_name_;
      std::map<std::string, GazeboExtension* > gazebo_extensions_;

  };
}

#endif
