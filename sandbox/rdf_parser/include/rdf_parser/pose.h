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

#ifndef RDF_PARSER_POSE_H
#define RDF_PARSER_POSE_H

#include <string>
#include <vector>
#include <math.h>
#include <boost/algorithm/string.hpp>

using namespace std;

namespace rdf_parser{

class Vector3
{
public:
  bool init(const std::string &vector_str)
  {
    std::vector<std::string> pieces;
    std::vector<double> xyz;
    boost::split( pieces, vector_str, boost::is_any_of(" "));
    for (unsigned int i = 0; i < pieces.size(); ++i){
      if (pieces[i] != ""){
        ///@todo: do better atof check if string is a number
        xyz.push_back(atof(pieces[i].c_str()));
      }
    }

    if (xyz.size() != 3) {
      cerr << "Vector did not contain 3 pieces:" << endl; 
      for (unsigned int i = 0; i < xyz.size(); ++i)
        cerr << "  " << i << ": '" << xyz[i] << "'" << endl;
      return false;
    }

    x = xyz[0];
    y = xyz[1];
    z = xyz[2];

    return true;
  };
  double x;
  double y;
  double z;
private:

};

class Rotation
{
public:
  bool init(const std::string &rotation_str)
  {
    Vector3 rpy;
    
    if (!rpy.init(rotation_str))
      return false;
    else
    {
      this->setFromRPY(rpy.x,rpy.y,rpy.z);
      return true;
    }
      
  };

  void getQuaternion(double &w,double &x,double &y,double &z)
  {
    w = this->w_;
    x = this->x_;
    y = this->y_;
    z = this->z_;
  };
  void getRPY(double &roll,double &pitch,double &yaw)
  {
    double sqw;
    double sqx;
    double sqy;
    double sqz;

    sqw = this->w_ * this->w_;
    sqx = this->x_ * this->x_;
    sqy = this->y_ * this->y_;
    sqz = this->z_ * this->z_;

    roll  = atan2(2 * (this->y_*this->z_ + this->w_*this->x_), sqw - sqx - sqy + sqz);
    pitch = asin(-2 * (this->x_*this->z_ - this->w_ * this->y_));
    yaw   = atan2(2 * (this->x_*this->y_ + this->w_*this->z_), sqw + sqx - sqy - sqz);

  };
  void setFromQuaternion(double w,double x,double y,double z)
  {
    this->w_ = w;
    this->x_ = x;
    this->y_ = y;
    this->z_ = z;
    this->normalize();
  };
  void setFromRPY(double roll, double pitch, double yaw)
  {
    double phi, the, psi;

    phi = roll / 2.0;
    the = pitch / 2.0;
    psi = yaw / 2.0;

    this->w_ = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
    this->x_ = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
    this->y_ = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
    this->z_ = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);

    this->normalize();
  };

private:
  double w_,x_,y_,z_;

  void normalize()
  {
    double s = sqrt(this->w_ * this->w_ +
                    this->x_ * this->x_ +
                    this->y_ * this->y_ +
                    this->z_ * this->z_);
    this->w_ /= s;
    this->x_ /= s;
    this->y_ /= s;
    this->z_ /= s;
  };
};

class Pose
{
public:
  bool initXml(TiXmlElement* xml)
  {
    if (!xml)
    {
      std::cout << "parsing pose: xml empty" << std::endl;
      return false;
    }
    else
    {
      if (!this->pos_.init(xml->Attribute("xyz")))
      {
        std::cout << "parsing pose: no xyz" << std::endl;
        return false;
      }
      if (!this->rot_.init(xml->Attribute("rpy")))
      {
        std::cout << "parsing pose: no rpy" << std::endl;
        return false;
      }
      return true;
    }
  };
  Vector3  pos_;
  Rotation rot_;
};

}

#endif
