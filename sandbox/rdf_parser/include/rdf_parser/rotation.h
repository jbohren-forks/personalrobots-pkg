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

/* Author: John Hsu */
/* this class should switch to using bullet or equiv linear math library */

#ifndef RDF_PARSER_ROTATION_H
#define RDF_PARSER_ROTATION_H

#include <string>
#include <vector>

using namespace std;

namespace rdf_parser{

class Rotation
{
public:
  Rotation() {w_=1.0;x_=0;y_=0;z_=0;}; // init to identity

  getQuaternion(double &w,double &x,double &y,double &z)
  {
    w = this->w_;
    x = this->x_;
    y = this->y_;
    z = this->z_;
  };
  getRPY(double &roll,double &pitch,double &yaw)
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
  setFromQuaternion(double w,double x,double y,double z)
  {
    this->w_ = w;
    this->x_ = x;
    this->y_ = y;
    this->z_ = z;
    this->normalize();
  };
  void setFromRPY(double r, double p, double y)
  {
    double phi, the, psi;

    phi = r / 2.0;
    the = p / 2.0;
    psi = y / 2.0;

    q_.w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
    q_.x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
    q_.y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
    q_.z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);

    this->normalize();
  };

private:
  double w_,x_,y_,z_;

  normalize()
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

}

#endif
