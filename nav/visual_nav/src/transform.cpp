/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#include <visual_nav/transform.h>
#include <cmath>
#include <iostream>

namespace visual_nav
{

using std::ostream;

Pose transform (const Transform2D& trans, const Pose& pose)
{
  double c=cos(trans.theta);
  double s=sin(trans.theta);

  return Pose(pose.x*c - pose.y*s + trans.dx, pose.x*s + pose.y*c + trans.dy, trans.theta+pose.theta);
}

Transform2D getTransformBetween (const Pose& pose1, const Pose& pose2)
{
  double theta = pose2.theta - pose1.theta;
  double c=cos(theta);
  double s=sin(theta);

  return Transform2D (pose2.x - pose1.x*c + pose1.y*s, pose2.y - pose1.y*c - pose1.x*s, theta);
}


Transform2D inverse (const Transform2D& trans)
{
  double c=cos(trans.theta);
  double s=sin(trans.theta);

  return Transform2D (-c*trans.dx - s*trans.dy, s*trans.dx - c*trans.dy, -trans.theta);
}
  

ostream& operator<< (ostream& str, const Pose& c)
{
  str << "(" << c.x << ", " << c.y << ", " << c.theta << ")";
  return str;
}

ostream& operator<< (ostream& str, const Transform2D& c)
{
  str << "(" << c.dx << ", " << c.dy << ", " << c.theta << ")";
  return str;
}

bool operator== (const Pose& p1, const Pose& p2)
{
  const double pi=3.14159265;
  const double TOL=1e-8;
  double intpart;
  return (abs(p1.x-p2.x)<TOL) && (abs(p1.y-p2.y)<TOL) && (modf((p1.theta-p2.theta)/(2*pi), &intpart)<TOL);
}


} // namespace visual_nav

